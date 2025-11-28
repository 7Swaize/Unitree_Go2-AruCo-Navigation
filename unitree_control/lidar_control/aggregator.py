import collections
import math
import struct
import threading
import time
from typing import Any, Callable, Generator, List, Optional, Tuple

import numpy as np
import open3d as o3d

from unitree_control.core.control_modules import DogModule
from unitree_control.dds.dds_constants import DDS_TOPICS


# command to install open3d: pip install open3d -U
# dont use conda cause its depricated


# downsample params
SCAN_DOWNSAMPLE = 0.15   # for feature extraction / ICP input
MAP_VOXEL = 0.06         # voxel size for fused global map
KEYFRAME_TRANSLATION = 0.5  # meters: create a new keyframe after this much movement
KEYFRAME_INTERVAL = 1.0     # seconds minimum between keyframes
MIN_POINTS_KEYFRAME = 200   # minimum points to accept a keyframe

# registration params
ICP_MAX_DIST = 1.0
ICP_ITER = 50

# FPFH (for loop detection)
FPFH_RADIUS_NORMAL = 0.5
FPFH_RADIUS_FEATURE = 1.0
RANSAC_DISTANCE_THRESHOLD = 1.5
RANSAC_NUM_ITER = 400000
RANSAC_CONFIDENCE = 0.999

# loop closure scanning window and frequency
LOOP_SEARCH_EVERY_N_KEYFRAMES = 5
LOOP_SEARCH_RADIUS = 10.0  # meters: only search earlier keyframes within this distance

# optimization
OPTIMIZE_EVERY_N_KEYFRAMES = 10


_DATATYPES = {
    1: ('b', 1),  # INT8
    2: ('B', 1),  # UINT8
    3: ('h', 2),  # INT16
    4: ('H', 2),  # UINT16
    5: ('i', 4),  # INT32
    6: ('I', 4),  # UINT32
    7: ('f', 4),  # FLOAT32
    8: ('d', 8),  # FLOAT64
}

def _get_struct_fmt(is_bigendian: bool, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in sorted(fields, key=lambda f: int(f.offset)):
        if field_names is not None and field.name not in field_names:
            continue

        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset 

        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype)
            continue

        datatype_fmt, datatype_length = _DATATYPES[field.datatype]
        fmt += datatype_fmt * max(1, getattr(field, 'count', 1))
        offset += datatype_length * max(1, getattr(field, 'count', 1))

    return fmt


def _read_points(cloud, field_names=None, skip_nans=True) -> Generator[Tuple[Any, ...], None, None]:
    """
    Generator: returns a tuple (values...) for each point, fields appear in order of field_names.
    If field_names is None, returns all fields in message order.
    """
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    # Possibly check forced conversion into buffer using data_buf = memoryview(list(cloud.data))
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, bytes(cloud.data), math.isnan # type: ignore
    unpack_from = struct.Struct(fmt).unpack_from
    

    if skip_nans:
        for v in range(height):
            offset = row_step * v
            for u in range(width):
                p = unpack_from(bytearray(data), offset)
                
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
                offset += point_step
    
    else:
        for v in range(height):
            offset = row_step * v
            for u in range(width):
                yield unpack_from(data, offset)
                offset += point_step



class PointCloudDecoder:
    def __init__(self) -> None:
        pass

    def cloud_to_xyzintensity(self, cloud) -> Tuple[np.ndarray, Optional[np.ndarray]]:
        # attempt to read x,y,z,intensity if present, else x,y,z
        field_names = [f.name for f in cloud.fields]
        want = []

        if all(k in field_names for k in ('x','y','z','intensity')):
            want = ['x','y','z','intensity']
        elif all(k in field_names for k in ('x','y','z')):
            want = ['x','y','z']
        else:
            return np.empty((0,3)), None
        
        arr = np.asarray(list(_read_points(cloud, want, skip_nans=True)), dtype=np.float32)
        if arr.size == 0:
            return np.empty((0,3)), None 
        
        pts = arr[:, :3].astype(np.float64)
        intensity = None
        if arr.shape[1] >= 4:
            intensity = arr[:, 3].astype(np.float64)

        return pts, intensity
            


class HeightMapDecoder:
    def __init__(self) -> None:
        pass

    def heightmap_to_numpy(self, heightmap) -> np.ndarray:
        ox, oy = heightmap.origin[0], heightmap.origin[1]  # type: ignore
        res = heightmap.resolution

        points = []
        for i in range(heightmap.height):
            for j in range(heightmap.width):
                h = heightmap.data[i * heightmap.width + j] # type: ignore
                x = ox + j * res
                y = oy + i * res
                z = h

                points.append([x, y, z])

        points = np.array(points, dtype=np.float32)
        return points


Keyframe = collections.namedtuple('Keyframe', ['idx', 'time', 'pose', 'pcd', 'pcd_down', 'fpfh']) 

# Pipeline reading: https://www.open3d.org/docs/latest/tutorial/pipelines/global_registration.html
# OPEN 3D docs: https://www.open3d.org/docs/latest/introduction.html
# RANSAC + FPFH https://medium.com/@amnahhmohammed/gentle-introduction-to-point-cloud-registration-using-open3d-pt-2-18df4cb8b16c
# RANSAC: https://www.thinkautonomous.ai/blog/ransac-algorithm/
# ICP: https://www.youtube.com/watch?v=4uWSo8v3iQA
class PoseGraphSLAM:
    def __init__(self) -> None:
        self.keyframes: List[Keyframe] = []
        self.pose_graph = o3d.pipelines.registration.PoseGraph()
        self.optimized = False
    

    def add_keyframe(self, kf: Keyframe, edge_to_prev:Optional[o3d.pipelines.registration.PoseGraphEdge]=None):
        if not self.keyframes:
            # add first node at identity
            node = o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(kf.pose))
            self.pose_graph.nodes.append(node)
        else:
            node = o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(kf.pose))
            self.pose_graph.nodes.append(node)
            if edge_to_prev is not None:
                self.pose_graph.edges.append(edge_to_prev)

        self.keyframes.append(kf)


    def add_loop_edge(self, i_from:int, i_to:int, trans:np.ndarray, information=np.eye(6)*100):
        # Add a loop closure as an uncertain constraint (set uncertain=False if you trust it)
        edge = o3d.pipelines.registration.PoseGraphEdge(i_from, i_to, trans, information, uncertain=False)
        self.pose_graph.edges.append(edge)
    

    def optimize(self):
        # global optimization settings
        option = o3d.pipelines.registration.GlobalOptimizationOption(max_correspondence_distance=ICP_MAX_DIST,
                                                                     edge_prune_threshold=0.25,
                                                                     reference_node=0)
        method = o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt()
        criteria = o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria()
        o3d.pipelines.registration.global_optimization(self.pose_graph,
                                                      method,
                                                      o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
                                                      option)
        self.optimized = True


# -----------------------------
# Feature helpers (FPFH)
# -----------------------------
def preprocess_point_cloud(pcd: o3d.geometry.PointCloud):
    pcd_down = pcd.voxel_down_sample(SCAN_DOWNSAMPLE)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=FPFH_RADIUS_NORMAL, max_nn=30)) 
    pcd_fpfh = compute_fpfh(pcd_down, FPFH_RADIUS_FEATURE)

    return pcd_down, pcd_fpfh

def estimate_normals(pcd, radius):
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=30))

def compute_fpfh(pcd, radius):
    return o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=100)
    )


# -----------------------------
# Registration helpers
# -----------------------------
def run_icp(source_down, target_down, init=np.eye(4), max_dist=ICP_MAX_DIST):
    # ensure normals
    if not target_down.has_normals():
        estimate_normals(target_down, FPFH_RADIUS_NORMAL)
    if not source_down.has_normals():
        estimate_normals(source_down, FPFH_RADIUS_NORMAL)
    reg = o3d.pipelines.registration.registration_icp(
        source_down, target_down, max_dist, init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=ICP_ITER)
    )

    return reg

def ransac_initial_alignment(source_down, target_down, source_fpfh, target_fpfh):
    # Use RANSAC to get coarse transform candidate
    distance = RANSAC_DISTANCE_THRESHOLD
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, mutual_filter=True,
        max_correspondence_distance=distance,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        ransac_n=4,
        checkers=[o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                  o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance)],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(RANSAC_NUM_ITER, RANSAC_CONFIDENCE)
    )

    return result


class LIDARSLAM:
    def __init__(self):
        self.pg = PoseGraphSLAM()
        self.last_keyframe_pose = np.eye(4)
        self.last_keyframe_time = 0.0
        self.keyframe_idx = 0
        self.lock = threading.Lock()

        self.vis = o3d.visualization.Visualizer() # type: ignore
        self.vis.create_window("GO2 LiDAR SLAM", width=1400, height=900)
        self.geom_added = False

        self.global_map = o3d.geometry.PointCloud()


    def try_add_keyframe(self, pcd_full: o3d.geometry.PointCloud, intensity=None, timestamp=None):
        tnow = timestamp if timestamp is not None else time.time()
        if (tnow - self.last_keyframe_time) < KEYFRAME_INTERVAL and self.keyframe_idx > 0:
            return False
        
        pcd_down, pcd_fpfh = preprocess_point_cloud(pcd_full)
        if len(pcd_down.points) < MIN_POINTS_KEYFRAME:
            return False
        
        # create keyframe with current pose equal to identity if first or equal to last pose
        pose = np.eye(4) if self.keyframe_idx == 0 else self.last_keyframe_pose.copy()
        kf = Keyframe(self.keyframe_idx, tnow, pose, pcd_full, pcd_down, pcd_fpfh)
        
        if self.keyframe_idx == 0:
            self.pg.add_keyframe(kf, edge_to_prev=None)
            print("[SLAM] added initial keyframe idx=0")
        else:
            # create odometry edge: register current down -> previous down
            prev_kf = self.pg.keyframes[-1]
            # try icp from current to prev (we point-to-plane)
            reg = run_icp(pcd_down, prev_kf.pcd_down, init=np.eye(4))
            transformation = reg.transformation
            information = np.eye(6) * max(1.0, reg.fitness * 100.0)
            edge = o3d.pipelines.registration.PoseGraphEdge(self.keyframe_idx-1, self.keyframe_idx, transformation, information, uncertain=False)
            self.pg.add_keyframe(kf, edge_to_prev=edge)
            print(f"[SLAM] added keyframe idx={self.keyframe_idx} (icp fitness {reg.fitness:.4f})")

        self.keyframe_idx += 1
        self.last_keyframe_time = tnow

        if (self.keyframe_idx % LOOP_SEARCH_EVERY_N_KEYFRAMES) == 0:
            self.detect_loop_closures_for_recent()

        if (self.keyframe_idx % OPTIMIZE_EVERY_N_KEYFRAMES) == 0:
            print("[SLAM] Running global optimization ...")
            self.pg.optimize()
            print("[SLAM] Optimization done. Reconstructing global map...")
            self.reconstruct_global_map()

        return True




    def detect_loop_closures_for_recent(self):
        """Search older keyframes for loop closures with the newest keyframe."""
        if len(self.pg.keyframes) < 2:
            return
        
        cur_idx = len(self.pg.keyframes) - 1
        cur_kf = self.pg.keyframes[cur_idx]
        # naive spatial filter: compare to all older keyframes (could index by position)
        for i in range(cur_idx - 2):
            # optionally check linear distance between poses (if pose exists)
            # compute coarse distance by comparing centroid distances
            other_kf = self.pg.keyframes[i]
            # compute approximate centroids and skip too-far ones
            c1 = np.mean(np.asarray(cur_kf.pcd_down.points), axis=0)
            c2 = np.mean(np.asarray(other_kf.pcd_down.points), axis=0)
            if np.linalg.norm(c1 - c2) > LOOP_SEARCH_RADIUS:
                continue

            # RANSAC initial alignment using FPFH
            ransac_res = ransac_initial_alignment(cur_kf.pcd_down, other_kf.pcd_down, cur_kf.fpfh, other_kf.fpfh)
            if ransac_res.fitness < 0.02:  # too weak
                continue
            # refine with ICP
            icp_res = run_icp(cur_kf.pcd_down, other_kf.pcd_down, init=ransac_res.transformation, max_dist=ICP_MAX_DIST*1.2)
            if icp_res.fitness > 0.15:
                # good loop closure: add constraint to pose graph
                info = np.eye(6) * max(1.0, icp_res.fitness * 100.0)
                self.pg.add_loop_edge(i, cur_idx, icp_res.transformation, information=info)

        print("[SLAM] Loop detection done.")



    def reconstruct_global_map(self):
        """Rebuild fused map from optimized pose graph nodes and keyframe point clouds."""
        fused = o3d.geometry.PointCloud()
        for idx, kf in enumerate(self.pg.keyframes):
            if idx < len(self.pg.pose_graph.nodes):
                node_pose_inv = self.pg.pose_graph.nodes[idx].pose  # this is inverse of node pose in Open3D's convention
                node_pose = np.linalg.inv(node_pose_inv)
            else:
                node_pose = kf.pose

            # transform keyframe full cloud to global
            pcd = kf.pcd.transform(node_pose)
            fused += pcd

        fused = fused.voxel_down_sample(MAP_VOXEL)
        self.global_map = fused

        # update visual
        with self.lock:
            if not self.geom_added:
                self.vis.add_geometry(self.global_map)
                self.geom_added = True
            else:
                self.vis.update_geometry(self.global_map)

            self.vis.poll_events()
            self.vis.update_renderer()


    def run_visualization_loop(self):
        """Main visualizer loop (runs in main thread)"""
        try:
            while True:
                with self.lock:
                    if self.geom_added:
                        self.vis.update_geometry(self.global_map)
                self.vis.poll_events()
                self.vis.update_renderer()
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("Visualizer interrupted.")
        finally:
            self.vis.destroy_window()



class LIDARModule(DogModule):
    def __init__(self, use_sdk: bool = True, visualize_lidar: bool = True):
        super().__init__("LIDAR")
        self.use_sdk = use_sdk
        self._visualize_lidar = visualize_lidar

        self.initialize()

    def initialize(self) -> None:
        if self._initialized or not self.use_sdk:
            return
        
        from unitree_sdk2py.core.channel import ChannelSubscriber
        from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_, PointField_
        from unitree_sdk2py.idl.unitree_go.msg.dds_ import HeightMap_

        self._point_cloud_listeners: List[Callable[[np.ndarray, Optional[np.ndarray]], None]] = []
        self._heightmap_listeners: List[Callable[[np.ndarray], None]] = []

        self._point_cloud_decoder = PointCloudDecoder()
        self._heightmap_decoder = HeightMapDecoder()
        self._slam = LIDARSLAM()
        
        self._point_cloud_dds_subscriber = ChannelSubscriber(DDS_TOPICS["ODOMETRY_LIDAR"], PointCloud2_)
        self._point_cloud_dds_subscriber.Init(self._point_cloud_callback, 10)

        self._heightmap_dds_subscriber = ChannelSubscriber(DDS_TOPICS["HEIGHT_MAP_ARRAY"], HeightMap_)
        self._heightmap_dds_subscriber.Init(self._heightmap_callback, 10)
        
    
    def _point_cloud_callback(self, cloud):
        pts, intensity = self._point_cloud_decoder.cloud_to_xyzintensity(cloud)
        if pts.shape[0] == 0:
            return
        
        for listener in self._point_cloud_listeners:
            try:
                listener(pts.copy(), intensity.copy() if intensity is not None else None)
            except Exception as e:
                print(f"Error in point cloud point listener: {e}")
        
        if self._visualize_lidar:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(pts)

            if intensity is not None:
                colors = np.asarray(intensity).astype(np.float64)
                colors -= colors.min()
                if colors.max() > 0:
                    colors /= colors.max()
                
                colors = np.stack([colors, colors, colors], axis=1)
                pcd.colors = o3d.utility.Vector3dVector(colors)

            self._slam.try_add_keyframe(pcd, intensity=intensity, timestamp=time.time())

    
    def _heightmap_callback(self, heightmap):
        pts = self._heightmap_decoder.heightmap_to_numpy(heightmap)
        if pts.shape[0] == 0:
            return

        for listener in self._heightmap_listeners:
            try:
                listener(pts.copy())
            except Exception as e:
                print(f"Error in heightmap listener: {e}")

        

    def register_point_cloud_listener(self, callback: Callable[[np.ndarray, Optional[np.ndarray]], None]) -> None:
        """
        Register a callback to receive decoded point cloud data.
        
        Args:
            callback: Function that takes (points, intensity) where:
                - points: np.ndarray of shape (N, 3) containing XYZ coordinates
                - intensity: Optional np.ndarray of shape (N, 1) containing intensity values
        """
        if callback not in self._point_cloud_listeners:
            self._point_cloud_listeners.append(callback)

    def unregister_point_cloud_listener(self, callback: Callable[[np.ndarray, Optional[np.ndarray]], None]) -> None:
        """
        Unregister a previously registered point cloud callback.
        
        Args:
            callback: The callback function to remove
        """
        if callback in self._point_cloud_listeners:
            self._point_cloud_listeners.remove(callback)

    def register_heightmap_listener(self, callback: Callable[[np.ndarray], None]) -> None:
        """
        Register a callback to receive decoded heightmap data.
        
        Args:
            callback: Function that takes (heightmap) where:
                - heightmap: np.ndarray of shape (N, 3) containing the decoded heightmap points
        """
        if callback not in self._heightmap_listeners:
            self._heightmap_listeners.append(callback)

    def unregister_heightmap_listener(self, callback: Callable[[np.ndarray], None]) -> None:
        """
        Unregister a previously registered heightmap callback.
        
        Args:
            callback: The callback function to remove
        """
        if callback in self._heightmap_listeners:
            self._heightmap_listeners.remove(callback)

    def clear_all_listeners(self) -> None:
        """Clear all registered hooks."""
        self._point_cloud_listeners.clear()
        self._heightmap_listeners.clear()


    def shutdown(self) -> None:
        if self._point_cloud_dds_subscriber:
            self._point_cloud_dds_subscriber.Close()
        if self._heightmap_dds_subscriber:
            self._heightmap_dds_subscriber.Close()

        self.clear_all_listeners()
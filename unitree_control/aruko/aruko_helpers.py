import math
import cv2

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_100)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

def extract_corners_points(corners):
    """Extract the four corner points from ArUco marker corners."""
    x0, y0 = corners[0]
    x1, y1 = corners[1]
    x2, y2 = corners[2]
    x3, y3 = corners[3]
    return (x0, y0), (x1, y1), (x2, y2), (x3, y3)


def determine_horizontal_offset(image, fiducial_center, screen_width):
    """Calculate horizontal offset from center of image."""
    midX = screen_width // 2
    cx = int(fiducial_center[0])
    return midX - cx


def compute_fiducial_center_point(p0, p1):
    """Compute center point between two diagonal corners."""
    mx = int((p0[0] + p1[0]) / 2)
    my = int((p0[1] + p1[1]) / 2)
    return (mx, my)


def dist_between_points(a, b):
    """Calculate distance between two points."""
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.sqrt(dx*dx + dy*dy)


def get_fiducial_area_from_corners(c0, c1, c2, c3):
    """Calculate approximate area of the marker."""
    width = dist_between_points(c0, c1)
    height = dist_between_points(c1, c2)
    return width * height


def get_aruko_marker(image):
    """Detect ArUco markers in an image."""
    if len(image.shape) == 2: 
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    elif image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)
    return corners, ids, rejected


def extract_data_from_marker(image, corners, ids):
    """Extract all relevant data from detected marker."""
    if corners is None or len(corners) == 0 or ids is None or len(ids) == 0:
        return -1, [(0, 0)], (0, 0), 0, 0

    bounds = [(int(x), int(y)) for x, y in corners[0][0]]
    marker_id = int(ids[0][0])

    c0, c1, c2, c3 = extract_corners_points(corners[0][0])
    fiducial_center = compute_fiducial_center_point(c0, c2)
    h_offset = determine_horizontal_offset(image, fiducial_center, image.shape[1])
    fiducial_area = get_fiducial_area_from_corners(c0, c1, c2, c3)

    return marker_id, bounds, fiducial_center, h_offset, fiducial_area
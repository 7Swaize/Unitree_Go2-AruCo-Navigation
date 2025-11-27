import builtins
import inspect
import os

# Global env blocker
# Put it in here: <venv>/lib/python3.x/site-packages/sitecustomize.py
# Or in here: /usr/lib/python3.x/site-packages/sitecustomize.py

_real_import = builtins.__import__

BLOCKED_MODULES = [
    "unitree_sdk",
]

STUDENT_DIRS = [
    "/home/saura/ros2_ws/test_scripts/",
]

def is_student_file(path: str) -> bool:
    """Return True if a file path belongs to student code."""
    if not path:
        return False

    ap = os.path.abspath(path)
    for sdir in STUDENT_DIRS:
        if ap.startswith(os.path.abspath(sdir)):
            return True
        
    return False


def _blocked_import(name, globals=None, locals=None, fromlist=(), level=0):
    # Get stack frames to see who is importing the module
    for frame in inspect.stack():
        caller = frame.filename

        if is_student_file(caller):
            for prefix in BLOCKED_MODULES:
                if name.startswith(prefix):
                    raise ImportError(
                        f"Direct import of '{name}' is blocked for students. "
                        "Use FunctionalityWrapper instead."
                    )

    return _real_import(name, globals, locals, fromlist, level)


builtins.__import__ = _blocked_import
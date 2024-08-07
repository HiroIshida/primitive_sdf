# _psdf.pyi

from typing import Any, List

import numpy as np

class Pose:
    def __init__(self, translation: np.ndarray, rotation: np.ndarray) -> None: ...

class AABB:
    lb: np.ndarray
    ub: np.ndarray

class SDFBase:
    def evaluate_batch(self, points: np.ndarray) -> np.ndarray:
        """Evaluate the SDF at the given points.
        Args:
            points: The (3, n_pts) points to evaluate the SDF at.
                TODO: Should this be a (n_pts, 3) array for consistency with numpy?
                This would require changing Eigen's default storage order to be row-major.
        Returns:
            The signed distance at each point.
        """
    ...

    def evaluate(self, point: np.ndarray) -> float:
        """Evaluate the SDF at a single point.
        Args:
            point: The (3,) point to evaluate the SDF at.
        Returns:
            The signed distance at the point.
        """
    def is_outside(self, point: np.ndarray) -> bool:
        """Check if a point is outside the shape.
        Args:
            point: The (3,) point to check.
        Returns:
            True if the point is outside, False otherwise.
        """
    def get_aabb(self) -> AABB:
        """Get the axis-aligned bounding box of the SDF.
        Returns:
            The axis-aligned bounding box.
        """

class UnionSDF(SDFBase):
    def __init__(self, sdf_list: List[SDFBase], create_bvh: bool) -> None: ...

class BoxSDF(SDFBase):
    def __init__(self, size: np.ndarray, pose: Pose) -> None: ...

class CylinderSDF(SDFBase):
    def __init__(self, radius: float, height: float, pose: Pose) -> None: ...

class SphereSDF(SDFBase):
    def __init__(self, radius: float, pose: Pose) -> None: ...

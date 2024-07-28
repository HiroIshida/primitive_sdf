import time
import numpy as np
from psdf import BoxSDF, CylinderSDF, UnionSDF, Pose
pose = Pose(np.ones(3), np.eye(3))  # trans and rot mat
sdf1 = BoxSDF(np.ones(3), pose)
sdf2 = CylinderSDF(1, 1, pose)
sdf = UnionSDF([sdf1, sdf2])
P = np.random.randn(3, 100)

ts = time.time()
for _ in range(1000):
    values = sdf.evaluate_batch(P)
print((time.time() - ts) / 1000 / 1e-6)  # micros

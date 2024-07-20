import time
import numpy as np
import build.primitive_sdf as psdf

from skrobot.model.primitives import Box

if __name__ == "__main__":
    tf = psdf.Pose(np.zeros(3), np.eye(3))
    sdf = psdf.BoxSDF(np.array([1, 1, 1]), tf)
    box = Box([1, 1, 1], with_sdf=True)
    points = np.random.randn(30, 3)

    ts = time.time()
    for _ in range(10000):
        vals = sdf.evaluate(points.T)
    print("c++ time: ", time.time() - ts)

    ts = time.time()
    for _ in range(10000):
        vals_sk = box.sdf(points)
    print("python time: ", time.time() - ts)

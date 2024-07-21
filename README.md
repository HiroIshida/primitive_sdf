## primitive_sdf
Collection of primitive SDFs written in c++, targeted for use from Python.

## Installation
```bash
sudo apt install libeigen3-dev
pip3 install scikit-build
pip3 install -e . -v
```

## Usage

```python
import numpy as np
from psdf import BoxSDF, CylinderSDF, UnionSDF, Pose
pose = Pose(np.ones(3), np.eye(3))  # trans and rot mat
sdf1 = BoxSDF(np.ones(3), pose)
sdf2 = CylinderSDF(1, 1, pose)
sdf = UnionSDF([sdf1, sdf2])
values = sdf.evaluate(np.random.randn(3, 1000))
```

## primitive_sdf [![PyPI version](https://badge.fury.io/py/psdf.svg)](https://pypi.org/project/psdf/) [![build_and_test](https://github.com/HiroIshida/primitive_sdf/actions/workflows/build_and_test.yaml/badge.svg)](https://github.com/HiroIshida/primitive_sdf/actions/workflows/build_and_test.yaml)
Collection of primitive SDFs written in c++, targeted for use from Python.

## Installation
From pypi
```bash
sudo apt install libeigen3-dev
pip3 install scikit-build
pip3 install psdf
```

From source
```bash
pip3 install -e . -v  # instad of pip3 install psdf
```

## Usage

```python
import numpy as np
from psdf import BoxSDF, CylinderSDF, UnionSDF, Pose
pose = Pose(np.ones(3), np.eye(3))  # trans and rot mat
sdf1 = BoxSDF(np.ones(3), pose)
sdf2 = CylinderSDF(1, 1, pose)
sdf = UnionSDF([sdf1, sdf2])
values = sdf.evaluate_batch(np.random.randn(3, 1000))
```

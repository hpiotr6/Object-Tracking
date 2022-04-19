# Object tracking

Tracker is a Python library for multiple object tracking, i.e. vehicles, based on cartesian plane coordinates. Kalman Filter was used, incorporating three main steps:
* Prediction of object's future location
* Reduction of noise introduced by inaccurate detections
* Facilitating the process of association of multiple objects to their tracks 

## Installation

Use the package manager [pip](https://pip.pypa.io/en/stable/) to install tracker.

```bash
pip install .
```

## Usage

```python
import tracker

# create obstacle from coordinates
self.tracker = Tracker()
self.tracker.detections = [(2, 3), (7, 1)]

```

## License
[MIT](https://choosealicense.com/licenses/mit/)

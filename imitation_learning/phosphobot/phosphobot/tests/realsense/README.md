### Run Realsense Camera with Python

This folder contains test scripts for the stereoscopic camera. We have the following model: intel realsense D405

Create a virtual environment with python version between 3.8 and 3.11 included

```bash
python -m venv .stereo
source .stereo/bin/activate
```

install via pip:
`pip install pyrealsense2-macosx`

You can find corresponding repository [here](https://github.com/cansik/pyrealsense2-macosx)

python scripts in this folder can display depth video or classic video video with depth video
They directly from:
https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.html

**Warning**: please run the python script using sudo !

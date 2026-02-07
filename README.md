# 1. Install & Setup
Setup udev rules
``` bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```
Installation
``` bash
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
pip install depthai
```

# 2. Current Plan
1. [Official Luxonis Method (May Not Work Well in Water)](https://github.com/luxonis/oak-examples/tree/main/depth-measurement/3d-measurement)
2. [Blue Robotics Advice (More Work)](https://jackmead515.github.io/notebooks/stereographic_depth_estimation_opencv.html)
3. [Blue Robotics Actual Software](https://gitlab.com/marinesitu-public/madrona/madrona)

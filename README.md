**Install & Setup**
Setup udev rules
``` bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Install dependencies
``` bash
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
```

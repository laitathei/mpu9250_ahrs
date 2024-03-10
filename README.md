# mpu9250_madgwick

## System Requirement
```
Ubuntu: 20.04/22.04
Python: 3.10.12
ROS: noetic
ROS2: humble
```

## Install Dependencies
```
chmod +x install.sh
sudo ./install.sh
pip3 install -r requirements.txt
```

## System Privileges
```
sudo cp 99-i2c.rules /etc/udev/rules.d
sudo udevadm control --reload
sudo reboot
```


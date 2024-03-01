# mpu9250_madgwick

## ROS2 installation
```
cd catkin_ws/src
git clone 
cd ..
colcon build --packages-select mpu9250_madgwick
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

## Start Program
```
ros2 run mpu9250_madgwick ros2_node
```
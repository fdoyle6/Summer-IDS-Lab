# The UDSSC Car Package

This package was made to be used with cars deployed in the **UD Scaled Smart City**.

### Setup
Connect the pi to a monitor/keyboard/mouse and run the following command:
```
sudo raspi-config
```
Use the options in `raspi-config` to enable both the camera and SSH.

###  Install
To install the package, navigate to your catkin_ws src folder. By default:
```
cd ~/catkin_ws/src
```
Then install by cloning the repository
```
git clone https://github.com/rayzayas/udssc-car.git
```
Now the car needs to be configured. Run the setup using:
```
cd ~/catkin_ws
catkin build
source ~/catkin_ws/devel/setup.bash
rosrun udssc_car config_car
```
Enter the car's hostname, as well as the wifi password. Reboot.

Now we need to ensure that the ROS environment variables are properly set.

Use the following commands:
```
export ROS_MASTER_URI=http://udssc-mainframe:11311/
export ROS_HOSTNAME=$HOSTNAME
```

Double check with:
```
echo $ROS_MASTER_URI
echo $ROS_HOSTNAME
```

Finally, make sure the relevant launch file is run on startup by adding the command to /etc/rc.local

Open the startup file by running:
```
sudo nano /etc/rc.local
```
Then add the relevant commands. For example:
```
roslaunch udssc_car muskrat_startup.launch
```

### Usage
If the installation instructions are followed, the commands entered into rc.local are run at startup.

So simply start the car and go!

### Troubleshooting
If `catkin build` gives you issues, please open a ticket in issues.

If the car isn't working beyond that, make sure you've tried the following things:
- Check if you can ping the car's IP from the mainframe using `ping car_ip`
- Check if the car has its ROS_MASTER_URI set up correctly (`echo $ROS_MASTER_URI` should print http://mainframe_ip:11311/)
- Make sure the ROS_IP is properly set (`echo $ROS_HOSTNAME` should print the hostname ie 'muskrat_##')
- Run `catkin_build` in the `catkin_ws` directory.
- Make sure the files in the scripts folder have execute privileges (`chmod +x script.py`)

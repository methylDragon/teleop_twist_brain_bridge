# teleop_twist_brain_bridge
Brain Teleoperation rosbridge Nodes for ROS!



## Requirements

- Python (preferably 3.6+)
- ROS on at least one computer in your local network



## Installing Dependencies

**ROS Dependencies**

Install these on your ROS computer:

```shell
sudo apt-get install -y ros-$ROS_DISTRO-rosbridge-server
sudo apt-get install -y ros-$ROS_DISTRO-tf2-web-republisher
```

### rosbridge

Install these on any computer that's running the rosbridge nodes:

```shell
pip install roslibpy
```

### EEG Dependencies

The individual folders might have READMEs that will include the various dependencies you need to install for integration with the various EEG clients. Make sure you read them!



## Running the Nodes

### Start the rosbridge Servers

On ROS computer:

```shell
# Terminal 1
roslaunch rosbridge_server rosbridge_websocket.launch

# Terminal 2 (if using TF)
rosrun tf2_web_republisher tf2_web_republisher
```

### Start the rosbridge Node

```shell
cd <NODE_DIRECTORY>
python <NODE_NAME>
```


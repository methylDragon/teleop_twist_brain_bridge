# Emotiv EEG Cortex2 Teleop rosbridge Nodes

Teleoperation node for the Emotiv Cortex2 wrapper for Emotiv EEGs



## Install Dependencies

```shell
pip install cortex2
```



## Configure

### Mental Command Node Configuration

File location: `config/command_config.yaml`

```yaml
emotiv_credentials:
  client_id: <CLIENT_ID_HERE>
  client_secret: <CLIENT_SECRET_HERE>
  
  # Name of the trained EmotivCortex2 profile for mental commands detection
  profile: <PROFILE NAME HERE>

  cortex_url: wss://localhost:6868 # (Default: wss://localhost:6868)

rosbridge_config:
  ros_uri: localhost # (Default: localhost)

teleop_config:
  # Mental commands under this value are set to 0 (Default: 0.2)
  command_min_threshold: 0.2

  # Maximum time since latest command before braking (Default: 0.5)
  command_time_threshold: 0.5

  # If neutral command is value is higher than this, override, and brake (Default: 0.5)
  neutral_command_override_threshold: 0.5

  # Size of data stream deques to use (Default: 10)
  data_deque_size: 10

  # ROS Published Topic (Default: /cmd_vel)
  published_topic: /cmd_vel

  max_lin_vel: 1.0 # (Default: 1.0)
  max_ang_vel: 0.5 # (Default: 0.5)
  publish_rate: 20 # In Hz (Default: 20)
```



## Example Usage

```shell
# Terminal 1
roslaunch rosbridge_server rosbridge_websocket.launch

# Terminal 2 (if using TF)
rosrun tf2_web_republisher tf2_web_republisher

# Terminal 3 (you might need to specify python3)
# This one runs the teleop rosbridge node for interfacing with mental commands!
python src/teleop_twist_cortex2_command_bridge.py
```
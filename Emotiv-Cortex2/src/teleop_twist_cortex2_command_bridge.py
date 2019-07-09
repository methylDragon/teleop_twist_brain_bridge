"""
teleop_twist_brain_bridge : teleop_twist_cortex2_command_bridge
@author: methylDragon
                                   .     .
                                .  |\-^-/|  .
                               /| } O.=.O { |\
                              /´ \ \_ ~ _/ / `\
                            /´ |  \-/ ~ \-/  | `\
                            |   |  /\\ //\  |   |
                             \|\|\/-""-""-\/|/|/
                                     ______/ /
                                     '------'
                       _   _        _  ___
             _ __  ___| |_| |_ _  _| ||   \ _ _ __ _ __ _ ___ _ _
            | '  \/ -_)  _| ' \ || | || |) | '_/ _` / _` / _ \ ' \
            |_|_|_\___|\__|_||_\_, |_||___/|_| \__,_\__, \___/_||_|
                               |__/                 |___/
            -------------------------------------------------------
                           github.com/methylDragon

    Description:
    rosbridge Teleoperation Node using the Emotiv EEG Cortex 2 API via Mental
    Commands!

    Requires:
    Python 3.6 or above
"""

from cortex2 import EmotivCortex2Client
import roslibpy
import getpass
import yaml
import time
import os

# Change Directory to filepath
try:
    dir_path = os.path.dirname(os.path.realpath(__file__))
except:
    dir_path = os.path.abspath('')

os.chdir(dir_path)

################################################################################
# OBTAIN CONFIGURATIONS
################################################################################

try:
    config = {}
    with open("../config/command_config.yaml", "r") as f:
        config = yaml.safe_load(f)
except:
    print("No configuration file found!")

if config.get('emotiv_credentials') is not None:
    client_id = config.get('emotiv_credentials').get('client_id')
    client_secret = config.get('emotiv_credentials').get('client_secret')
    cortex_url = config.get('emotiv_credentials').get('client_secret', "wss://localhost:6868")
    profile = config.get('emotiv_credentials').get('profile')

if config.get('rosbridge_config') is not None:
    ros_uri = config.get('rosbridge_config').get('ros_uri', "localhost")

if config.get('teleop_config') is not None:
    command_min_threshold = config.get('teleop_config').get('command_min_threshold', 0.2)
    command_time_threshold = config.get('teleop_config').get('command_time_threshold', 0.5)
    neutral_command_override_threshold = config.get('teleop_config').get('neutral_command_override_threshold', 0.5)
    data_deque_size = int(config.get('teleop_config').get('data_deque_size', 10))
    published_topic = config.get('teleop_config').get('published_topic', "/cmd_vel")
    max_lin_vel = config.get('teleop_config').get('max_lin_vel', 1.0)
    max_ang_vel = config.get('teleop_config').get('max_ang_vel', 0.5)
    publish_rate = config.get('teleop_config').get('publish_rate', 20)

while client_id is None:
    client_id = input("Input Emotiv client_id: ")

while client_secret is None:
    client_secret = getpass.getpass(prompt="Input Emotiv client_secret: ")

while profile is None:
    profile = input("Input Emotiv profile to use: ")

################################################################################
# START CLIENTS AND SETUP
################################################################################

# Start Emotiv Client
print("\nStarting Emotiv Cortex2 Client...\n")
cortex_client = EmotivCortex2Client(cortex_url,
                                    authenticate=True,
                                    client_id=client_id,
                                    client_secret=client_secret,
                                    data_deque_size=data_deque_size,
                                    check_response=True,
                                    debug=False)
print("Emotiv Cortex2 Client connected!")

# Start rosbridge Client
print("\nConnecting to rosbridge server at: %s" % ros_uri)
print("Please ensure ROS master and rosbridge server is running!")

rosbridge_client = roslibpy.Ros(host=ros_uri, port=9090)
rosbridge_client.run()
print("rosbridge server connected!")
rosbridge_client.is_connected

# Connect Emotiv EEG Headset and create a session
cortex_client.request_access()
cortex_client.query_headsets()
cortex_client.connect_headset(0)
cortex_client.create_session(0)
cortex_client.query_profiles()
cortex_client.load_profile(profile, 0)

# Subscribe to mental commands stream
cortex_client.subscribe(streams=["com"])
print("Emotiv EEG Mental Commands Subscriber Spinning!")

# Set up Exponential Moving Average Weights
incoming_weight = 1.0 / cortex_client.data_deque_size
average_weight = 1.0 - incoming_weight

# Advertise cmd_vel topic
cmd_vel_pub = roslibpy.Topic(rosbridge_client, published_topic, 'geometry_msgs/Twist')

################################################################################
# FUNCTIONS
################################################################################

def construct_cmd_vel_msg(vel_x, vel_y, vel_z, ang_x, ang_y, ang_z):
    '''Construct a cmd_vel message.'''
    return roslibpy.Message({'linear':{'x': vel_x, 'y': vel_y, 'z': vel_z},
                             'angular':{'x': ang_x, 'y': ang_y, 'z': ang_z}})

def clamp_value(value, clamp):
    '''Force value magnitude to 0 if smaller than clamp.'''
    if abs(value) < clamp:
        return 0
    else:
        return value

def remap(value, orig_low, orig_high, remapped_low, remapped_high):
    '''Remap value from original scale to new scale.'''
    return (remapped_low
            + (value - orig_low)
            * (remapped_high - remapped_low) / (orig_high - orig_low))

################################################################################
# CORE LOOP
################################################################################

# Initialise command values
command_values = {"neutral": 0,
                  "push": 0,
                  "pull": 0,
                  "lift": 0,
                  "drop": 0,
                  "left": 0,
                  "right": 0,
                  "rotateLeft": 0,
                  "rotateRight": 0,
                  "rotateClockwise": 0,
                  "rotateCounterClockwise": 0,
                  "rotateForwards": 0,
                  "rotateReverse": 0,
                  "disappear": 0}

while rosbridge_client.is_connected:
    try:
        # Pop off the front of the stream
        latest_data = list(cortex_client.data_streams.values())[0]['com'].popleft()
        command_name = latest_data['data']['com'][0]
        command_value = latest_data['data']['com'][1]

        # Apply Exponential Moving Average for the command here
        command_values[command_name] = (incoming_weight * command_value
                                        + command_values[command_name] * average_weight)

        # Stop if neutral is high
        if command_values.get('neutral', 0) > neutral_command_override_threshold:
            cmd_vel_pub.publish(construct_cmd_vel_msg(0, 0, 0, 0, 0, 0))
            last_command_time = time.time()
            continue

        # Otherwise calculate cmd_vel commands

        # Clamped Forward - Clamped Reverse
        lin_bias = (clamp_value(max(
                                command_values.get('push', 0),
                                command_values.get('lift', 0),
                                command_values.get('rotateForwards', 0)
                                )
                            , command_min_threshold)
                - clamp_value(max(
                                command_values.get('pull', 0),
                                command_values.get('drop', 0),
                                command_values.get('rotateReverse', 0)
                                )
                            , command_min_threshold)
                    )

        # Clamped Left - Clamped Right
        ang_bias = (clamp_value(max(
                                command_values.get('left', 0),
                                command_values.get('rotateLeft', 0),
                                command_values.get('rotateCounterClockwise', 0)
                                )
                                , command_min_threshold)
                    - clamp_value(max(
                                command_values.get('right', 0),
                                command_values.get('rotateRight', 0),
                                command_values.get('rotateClockwise', 0)
                                )
                                , command_min_threshold)
                    )

        lin_x = remap(lin_bias, command_min_threshold, 1, 0, max_lin_vel)
        ang_z = remap(ang_bias, command_min_threshold, 1, 0, max_ang_vel)

        cmd_vel_pub.publish(construct_cmd_vel_msg(lin_x, 0, 0, 0, 0, ang_z))
        last_command_time = time.time()
        time.sleep(1 / publish_rate)
    except:
        # If no recent command has been sent, send a stop command
        if (time.time() - last_command_time) > command_time_threshold:
            print("No recent commands! Publishing stop command!")
            cmd_vel_pub.publish(construct_cmd_vel_msg(0, 0, 0, 0, 0, 0))

# Cleanup on exit
cortex_client.stop_subscriber()
rosbridge_client.terminate()

import socket
import math
import time
import sys
import json  # Add import for JSON

mysocket = socket.socket()
# mysocket.connect(('127.0.0.1', 12346))
mysocket.connect(('192.168.4.5',12345))


"""
# xArm-Python-SDK: https://github.com/xArm-Developer/xArm-Python-SDK
# git clone git@github.com:xArm-Developer/xArm-Python-SDK.git
# cd xArm-Python-SDK
# python setup.py install
"""
try:
    from xarm.tools import utils
except:
    pass
from xarm import version
from xarm.wrapper import XArmAPI

# Connect to xarm7 robot (update IP address as needed)
arm = XArmAPI('192.168.4.15')  # Update this IP to match your xarm7
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

# arm.reset(wait=True)

arm.set_mode(1)
arm.set_state(0)
time.sleep(0.1)

variables = {}
params = {'speed': 100, 'acc': 2000, 'angle_speed': 20, 'angle_acc': 500, 'events': {}, 'variables': variables, 'callback_in_thread': True, 'quit': False}


# Register error/warn changed callback
def error_warn_change_callback(data):#
    if data and data['error_code'] != 0:
        params['quit'] = True
        pprint('err={}, quit'.format(data['error_code']))
        arm.release_error_warn_changed_callback(error_warn_change_callback)
arm.register_error_warn_changed_callback(error_warn_change_callback)


# Register state changed callback
def state_changed_callback(data):
    if data and data['state'] == 4:
        if arm.version_number[0] >= 1 and arm.version_number[1] >= 1 and arm.version_number[2] > 0:
            params['quit'] = True
            pprint('state=4, quit')
            arm.release_state_changed_callback(state_changed_callback)
arm.register_state_changed_callback(state_changed_callback)


# Register counter value changed callback
if hasattr(arm, 'register_count_changed_callback'):
    def count_changed_callback(data):
        if not params['quit']:
            pprint('counter val: {}'.format(data['count']))
    arm.register_count_changed_callback(count_changed_callback)


# Register connect changed callback
def connect_changed_callback(data):
    if data and not data['connected']:
        params['quit'] = True
        pprint('disconnect, connected={}, reported={}, quit'.format(data['connected'], data['reported']))
        arm.release_connect_changed_callback(error_warn_change_callback)
arm.register_connect_changed_callback(error_warn_change_callback)


def close_socket(thissocket):
    try:
        thissocket.shutdown(socket.SHUT_RDWR)
        thissocket.close()
        thissocket = None
    except socket.error as e:
        pass
    print("socket is closed")


def is_moving():
    """Check if the robot is still moving."""    
    while arm.get_is_moving():
        time.sleep(0.1)

def move_robot(joints):
    # Ensure we have 7 joints for xarm7
    if len(joints) != 7:
        print(f"Error: Expected 7 joints for xarm7, got {len(joints)}")
        return False
    
    arm.set_servo_angle(angle=joints, is_radian=True, wait=True)
    # is_moving()  # Wait until movement is finished
    while arm.get_is_moving():
        time.sleep(0.1)
        print(".", end="")
        sys.stdout.flush()
    return True

firstConnect = True

try:
    while True:
        data = mysocket.recv(1024)
        message = data.decode()
        if message == "Done":
            break
        
        # Handle special commands
        if message == "GET_CURRENT_POSITION":
            print("Received GET_CURRENT_POSITION request")
            # Get current robot position and send it back
            try:
                current_joints = arm.get_servo_angle(is_radian=True)
                if current_joints and len(current_joints) == 7:
                    response = json.dumps(current_joints)
                    mysocket.send(response.encode())
                    print(f"Sent current joints: {current_joints}")
                else:
                    print("Warning: Could not get current robot position")
                    # Send default position
                    default_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                    mysocket.send(json.dumps(default_joints).encode())
            except Exception as e:
                print(f"Error getting robot position: {e}")
                # Send default position
                default_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                mysocket.send(json.dumps(default_joints).encode())
            continue
        
        # Parse joint data as JSON
        try:
            joints = json.loads(message)  # Parse joint data as JSON
        except json.JSONDecodeError:
            print("Error decoding JSON:", message)
            continue
        
        # print(joints)
        joints_deg = [math.degrees(joint) for joint in joints]
        
        if firstConnect:
            arm.set_mode(0)
            arm.set_state(state=0)
            print("moving to first position", joints)
            if move_robot(joints):
                firstConnect = False
                arm.set_mode(1)
                arm.set_state(0)
                mysocket.send(json.dumps("go").encode())  # Send acknowledgment as JSON
            else:
                print("Failed to move to first position")
                break
        else:
            try:
                if arm.connected and arm.state != 4:
                    # Ensure we have 7 joints for xarm7
                    if len(joints) == 7:
                        arm.set_servo_angle_j(joints, is_radian=True)
                    else:
                        print(f"Error: Expected 7 joints for xarm7, got {len(joints)}")
            except Exception as e:
                print("Error:", e)
                break       
        # print("moved to", joints_deg)
        
except KeyboardInterrupt:
    print("closing socket...")
    close_socket(mysocket)

print("Isaac Sim Connection Stopped")

if hasattr(arm, 'release_count_changed_callback'):
    arm.release_count_changed_callback(count_changed_callback)
arm.release_error_warn_changed_callback(state_changed_callback)
arm.release_state_changed_callback(state_changed_callback)
arm.release_connect_changed_callback(error_warn_change_callback)

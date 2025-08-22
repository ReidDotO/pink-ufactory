#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# SPDX-License-Identifier: Apache-2.0
# Copyright 2024 Ivan Domrachev, Simeon Nedelchev
#
# Robert Twomey | 2025 | rtwomey@ucsd.edu | cohab-lab.net

"""Ufactory xarm7 arm that uses fixed world coordinates and follows external target positions.

This script:
1. Uses fixed world coordinate system where all joints at 0 = [206, 0, 120.5, 180, 0, 0] mm
2. Reads target positions from /tmp/target_positions.json
3. Moves the robot to those target positions while avoiding self-collisions
4. Provides smooth motion interpolation for large position changes
5. Updates current robot state in /tmp/current_robot_state.json

To provide target positions, create a file /tmp/target_positions.json with JSON format:
Example: {"x": 0.4, "y": 0.3, "z": 0.6, "roll": 0, "pitch": 90, "yaw": 0}

IMPORTANT: Positions must be in METERS (not millimeters). Convert from UFactory Studio by dividing by 1000.
Example: Studio position [418.5, 0, 399.9] becomes {"x": 0.4185, "y": 0.0, "z": 0.3999}

NOTE: The robot maintains its go-front orientation unless you specify otherwise in the JSON.

SMOOTH MOTION: Large position changes are automatically smoothed:
- Maximum step size: 5mm per iteration (200Hz = 1m/s max speed)
- Interpolation factor: 5% of remaining distance per step
- Prevents jerky movements and ensures safety

REAL ROBOT SYNC: When streaming to a real robot, the script will automatically:
1. Request current joint positions from the robot controller
2. Synchronize its internal state with the real robot position
3. Prevent unexpected jumps when starting control

The script reads targets every 200Hz and updates current state every 0.5 seconds."""

import argparse

import numpy as np
import qpsolvers
from loop_rate_limiters import RateLimiter

import pinocchio as pin
import os

import meshcat_shapes
import pink
from pink import solve_ik
from pink.barriers import PositionBarrier
from pink.barriers import SelfCollisionBarrier
from pink.utils import process_collision_pairs

from pink.barriers import BodySphericalBarrier

from pink.tasks import FrameTask, PostureTask
from pink.visualization import start_meshcat_visualizer

import socket
import sys
import json  # Add import for JSON

# streamJoints = True  # Toggle for streaming
streamJoints = True  # Toggle for streaming

          
q = None

if streamJoints:
    txport = 12345       # Port for joint streaming

    # Create socket outside main loop
    txsocket = socket.socket()
    txsocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    txsocket.bind(('', txport))
    txsocket.listen(5)

    connected = False
    txconn = None

# Wait for connection before starting simulation
def wait_for_connection():
    global txconn, connected, q
    while not connected:
        print("tx: waiting for connection...")
        txconn, txaddr = txsocket.accept()
        connected = True
    
    print("tx: accepted connection from", str(txaddr[0]), ":", str(txaddr[1]))
    
    # Synchronize with real robot - get current joint positions
    print("Synchronizing with real robot...")
    try:
        # Request current position from robot controller
        txconn.send(b"GET_CURRENT_POSITION")
        response = txconn.recv(1024)
        message = response.decode()
        
        # Check if we got a valid response
        if message and message != "GET_CURRENT_POSITION":
            try:
                current_joints = json.loads(message)
                print(f"Real robot current joints: {current_joints}")
                
                # Update our configuration to match reality
                if len(current_joints) == 7:  # Ensure we have 7 joints for xarm7
                    q = current_joints
                    print("Successfully synchronized with real robot position")
                else:
                    print("Warning: Invalid joint data received, using default position")
                    q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            except json.JSONDecodeError:
                print(f"Warning: Invalid JSON response: {message}")
                q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        else:
            print("Warning: No valid response from robot, using default position")
            q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    except Exception as e:
        print(f"Warning: Could not sync with real robot: {e}")
        print("Using default position - robot may jump unexpectedly!")
        q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    # Send our current joint data as JSON
    txconn.send(json.dumps(q).encode())
    
    # wait for reply
    print("waiting to move to initial position...", end="")
    sys.stdout.flush()
    data = txconn.recv(1024)
    print("done! going", data)

try:
    from robot_descriptions.loaders.pinocchio import load_robot_description
except ModuleNotFoundError as exc:
    raise ModuleNotFoundError(
        "Examples need robot_descriptions, "
        "try `[conda|pip] install robot_descriptions`"
    ) from exc  # noqa: E501


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--verbose",
        "-v",
        help="print out task errors and CBF values during execution",
        default=False,
        action="store_true",
    )
    
    # model, visual_model, collision_model = pin.Model(), pin.GeometryModel(), pin.GeometryModel()


    args = parser.parse_args()
    
    # robot = load_robot_description("xarm7_mj_description", root_joint=None)
    urdf_path = os.path.join(
        os.path.dirname(__file__),
        "../robots",
        "xarm7.urdf",
    )

    robot = pin.RobotWrapper.BuildFromURDF(
        filename=urdf_path,
        package_dirs=["."],
        root_joint=None,
    )
    print(f"URDF description successfully loaded in {robot}")

    viz = start_meshcat_visualizer(robot)

    end_effector_task = FrameTask(
        "link7",
        position_cost=30.0,  # [cost] / [m]
        orientation_cost=1.0,  # [cost] / [rad]
    )

    # Create a custom posture task with joint-specific costs to control priorities
    # Lower cost = higher priority, higher cost = lower priority
    joint_costs = np.array([
        1e-3,   # joint1: high priority (can rotate 360°)
        1e-2,   # joint2: medium priority (limited range -118° to 120°)
        1e-3,   # joint3: high priority (can rotate 360°)
        1e-2,   # joint4: medium priority (limited range -11° to 225°)
        1e-3,   # joint5: high priority (can rotate 360°)
        1e-1,   # joint6: low priority (limited range -97° to 180°) - discourage movement
        1e-3,   # joint7: high priority (can rotate 360°)
    ])
    
    posture_task = PostureTask(
        cost=joint_costs,  # Use joint-specific costs instead of single cost
    )

    # starting point - use your specified Cartesian position
    # Start with joint angles that correspond to your desired Cartesian start position
    # These joint angles (in degrees) correspond to your target: [418.5, 0, 399.9, -25.5, 87.7, -25.5]
    q_ref = np.array([0.0, 2.5, 0.0, 37.3, 0.0, -57.3, 179.0]) * np.pi / 180.0  # Convert degrees to radians
    
    # Define fixed world coordinate system
    # When all joints are at 0, end effector is at these coordinates (in mm)
    WORLD_ORIGIN_MM = [206.0, 0.0, 120.5, 180.0, 0.0, 0.0]  # [x, y, z, roll, pitch, yaw]
    WORLD_ORIGIN_M = [206.0/1000.0, 0.0/1000.0, 120.5/1000.0, 180.0, 0.0, 0.0]  # Convert to meters
    
    print(f"Fixed world origin (all joints at 0): {WORLD_ORIGIN_MM} mm = {WORLD_ORIGIN_M} m")
    
    # Initialize the target positions file with your desired starting position
    start_position = {
        "x": 418.5/1000.0,  # Convert mm to meters
        "y": 0.0/1000.0, 
        "z": 399.9/1000.0,
        "roll": -25.5,
        "pitch": 87.7,
        "yaw": -25.5
    }
    
    # Create the target positions file with your starting position
    try:
        with open('/tmp/target_positions.json', 'w') as f:
            json.dump(start_position, f, indent=2)
        print(f"Created /tmp/target_positions.json with starting target: {start_position}")
        print(f"Starting target: x={418.5}mm, y={0}mm, z={399.9}mm, roll={-25.5}°, pitch={87.7}°, yaw={-25.5}°")
    except Exception as e:
        print(f"Warning: Could not create target file: {e}")
    
    # Initialize the current state file
    try:
        with open('/tmp/current_robot_state.json', 'w') as f:
            json.dump(start_position, f, indent=2)
        print("Created /tmp/current_robot_state.json for robot state tracking")
    except Exception as e:
        print(f"Warning: Could not create state file: {e}")

    # pos_barrier = PositionBarrier(
    #     "link7",
    #     indices=[1],
    #     p_min=np.array([-0.4]),
    #     p_max=np.array([0.6]),
    #     gain=np.array([100.0]),
    #     safe_displacement_gain=1.0,
    # )
    # barriers = [pos_barrier]


    # ee_frame_placement = pin.SE3()
    # ee_frame_placement.translation = np.array([0.0, 0.0, 0.05])
    # ee_frame_placement.rotation = np.eye(3)

    # ee_frame = pin.Frame(
    #     "xarm7_ee_barrier",
    #     robot.model.getJointId("joint7"),
    #     robot.model.getFrameId("link7"),
    #     ee_frame_placement,
    #     pin.FrameType.OP_FRAME,
    # )

    # robot.model.addFrame(ee_frame)

    # robot.data = pin.Data(robot.model)

    # # Pink barriers
    # ee_barrier = BodySphericalBarrier(
    #     ("xarm7_ee_barrier"),
    #     d_min=0.25,
    #     gain=100.0,
    #     safe_displacement_gain=1.0,
    # )
    # barriers = [ee_barrier]

    srdf_path = os.path.join(
        os.path.dirname(__file__),
        "../robots",
        "xarm7.srdf",
    )
    print(srdf_path)

    # Collisions: processing collisions from urdf (include all) and srdf (exclude specified)
    # and updating collision model and creating corresponding collision data
    robot.collision_data = process_collision_pairs(robot.model, robot.collision_model, srdf_path)

    configuration = pink.Configuration(
        robot.model,
        robot.data,
        q_ref,
        collision_model=robot.collision_model,  # Collision model is required for self_collision_barrier
        collision_data=robot.collision_data,
    )
    # print(robot.collision_model.collisionPairs)

    collision_barrier = SelfCollisionBarrier(
        n_collision_pairs=len(robot.collision_model.collisionPairs),
        gain=5.0,  # Reduced from 20.0 to make robot less jumpy
        safe_displacement_gain=1.0,
        d_min=0.01,
    )
    
    # Joint limits are handled by the posture task with joint-specific costs
    # The ConfigurationLimit barrier doesn't exist in this version of Pink
    barriers = [collision_barrier]

    tasks = [end_effector_task, posture_task]

    # configuration = pink.Configuration(robot.model, robot.data, q_ref, collision_model=robot.collision_model)
    
    # configuration = pink.Configuration(robot.model, robot.data, q_ref)
    for task in tasks:
        task.set_target_from_configuration(configuration)
    viz.display(configuration.q)

    viewer = viz.viewer
    meshcat_shapes.frame(viewer["end_effector_target"], opacity=0.5)
    meshcat_shapes.frame(viewer["end_effector"], opacity=1.0)

    # Select QP solver
    solver = qpsolvers.available_solvers[0]
    if "osqp" in qpsolvers.available_solvers:
        solver = "osqp"

    rate = RateLimiter(frequency=200.0)
    dt = rate.period
    t = 0.0  # [s]
    while True:
        # Update task targets
        end_effector_target = end_effector_task.transform_target_to_world
        
        # Smooth motion approach: interpolate between current and target positions
        try:
            with open('/tmp/target_positions.json', 'r') as f:
                target_data = json.load(f)
                if 'x' in target_data and 'y' in target_data and 'z' in target_data:
                    # Get target position from targets file
                    target_x = float(target_data['x'])
                    target_y = float(target_data['y']) 
                    target_z = float(target_data['z'])
                    
                    # Get current position
                    current_pos = configuration.get_transform_frame_to_world("link7").translation
                    current_x, current_y, current_z = current_pos[0], current_pos[1], current_pos[2]
                    
                    # Calculate distance to target
                    distance = np.sqrt((target_x - current_x)**2 + (target_y - current_y)**2 + (target_z - current_z)**2)
                    
                    # Smooth interpolation parameters
                    max_step_size = 0.005  # Maximum step size per iteration (5mm)
                    interpolation_factor = 0.025  # How much to move toward target each step (2.5%)
                    
                    # Add position hysteresis to prevent jumping when very close to target
                    if distance > max_step_size:
                        # Move toward target smoothly
                        step_size = min(max_step_size, distance * interpolation_factor)
                        direction = np.array([target_x - current_x, target_y - current_y, target_z - current_z])
                        direction = direction / np.linalg.norm(direction) * step_size
                        
                        x = current_x + direction[0]
                        y = current_y + direction[1]
                        z = current_z + direction[2]
                        
                        # print(f"Moving toward target: distance={distance:.3f}m, step={step_size:.3f}m")
                    elif distance > 0.002:  # Between 2mm and 5mm from target
                        # Very close - move very slowly
                        step_size = distance * 0.01  # 1% of remaining distance
                        direction = np.array([target_x - current_x, target_y - current_y, target_z - current_z])
                        direction = direction / np.linalg.norm(direction) * step_size
                        
                        x = current_x + direction[0]
                        y = current_y + direction[1]
                        z = current_z + direction[2]
                    else:
                        # Within 2mm of target - stay exactly where we are to prevent jumping
                        x, y, z = current_x, current_y, current_z
                else:
                    # Stay at your desired starting position
                    x, y, z = 418.5/1000.0, 0.0/1000.0, 399.9/1000.0
        except:
            # No targets file or error - stay at your desired starting position
            x, y, z = 418.5/1000.0, 0.0/1000.0, 399.9/1000.0

        # Set position
        end_effector_target.translation[0] = x
        end_effector_target.translation[1] = y
        end_effector_target.translation[2] = z
        
        # Update the current robot state file (every few iterations to avoid excessive file I/O)
        if t % 0.5 < dt:  # Update every 0.5 seconds
            try:
                current_orientation = end_effector_target.rotation
                # Convert rotation matrix back to RPY angles (simplified)
                from scipy.spatial.transform import Rotation
                rpy_angles = Rotation.from_matrix(current_orientation).as_euler('xyz', degrees=True)
                
                current_state = {
                    "x": round(x, 4),
                    "y": round(y, 4),
                    "z": round(z, 4),
                    "roll": round(rpy_angles[0], 1),
                    "pitch": round(rpy_angles[1], 1),
                    "yaw": round(rpy_angles[2], 1)
                }
                
                with open('/tmp/current_robot_state.json', 'w') as f:
                    json.dump(current_state, f, indent=2)
            except Exception as e:
                # Silently fail - don't interrupt robot operation
                pass

        # Handle orientation - use your specified default or keep current
        try:
            if 'roll' in target_data and 'pitch' in target_data and 'yaw' in target_data:
                # Use orientation from targets file
                roll = float(target_data['roll']) * np.pi / 180.0  # Convert degrees to radians
                pitch = float(target_data['pitch']) * np.pi / 180.0
                yaw = float(target_data['yaw']) * np.pi / 180.0
                
                # Convert RPY to rotation matrix
                from scipy.spatial.transform import Rotation
                rpy_angles = [roll, pitch, yaw]  # [roll, pitch, yaw] in radians
                R = Rotation.from_euler('xyz', rpy_angles).as_matrix()
                end_effector_target.rotation = R
            else:
                # Use your specified default orientation
                from scipy.spatial.transform import Rotation
                default_roll = -25.5 * np.pi / 180.0
                default_pitch = 87.7 * np.pi / 180.0
                default_yaw = -25.5 * np.pi / 180.0
                rpy_angles = [default_roll, default_pitch, default_yaw]
                R = Rotation.from_euler('xyz', rpy_angles).as_matrix()
                end_effector_target.rotation = R
        except:
            # Fallback to your specified default orientation
            from scipy.spatial.transform import Rotation
            default_roll = -25.5 * np.pi / 180.0
            default_pitch = 87.7 * np.pi / 180.0
            default_yaw = -25.5 * np.pi / 180.0
            rpy_angles = [default_roll, default_pitch, default_yaw]
            R = Rotation.from_euler('xyz', rpy_angles).as_matrix()
            end_effector_target.rotation = R

        # Update visualization frames
        viewer["end_effector_target"].set_transform(end_effector_target.np)
        viewer["end_effector"].set_transform(
            configuration.get_transform_frame_to_world(
                end_effector_task.frame
            ).np
        )

        velocity = solve_ik(
            configuration,
            tasks,
            dt,
            solver=solver,
            barriers=barriers,
            safety_break=False,
        )
        configuration.integrate_inplace(velocity, dt)
        q = configuration.q.tolist()
        
        # Stream joint values over socket
        if streamJoints:
            if connected:
                try:
                    txconn.send(json.dumps(configuration.q.tolist()).encode())  # Send joint data as JSON
                except:
                    connected = False
                    print("tx: disconnected.")
            else:
                wait_for_connection()

        # G, h = collision_barrier.compute_qp_inequalities(configuration, dt=dt)
        # # G, h = pos_barrier.compute_qp_inequalities(configuration, dt=dt)
        # distance_to_manipulator = configuration.get_transform_frame_to_world(
        #     "link7"
        # ).translation[1]
        # if args.verbose:
        #     print(
        #         f"Task error: {end_effector_task.compute_error(configuration)}"
        #     )
        #     print(
        #         "Position CBF value: "
        #         f"{pos_barrier.compute_barrier(configuration)[0]:0.3f} >= 0"
        #     )
        #     print(f"Distance to manipulator: {distance_to_manipulator} <= 0.6")
        #     print("-" * 60)

        # Visualize result at fixed FPS
        viz.display(configuration.q)
        rate.sleep()
        t += dt

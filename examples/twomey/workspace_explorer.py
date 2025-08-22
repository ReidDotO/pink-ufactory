#!/usr/bin/env python3
"""XArm7 Workspace Explorer - Find the robot's reachable workspace bounds."""

import numpy as np
import json
import time
import os

def test_position(x, y, z, roll=-25.5, pitch=87.7, yaw=-25.5):
    """Test if a position is reachable by updating the target JSON file."""
    target = {
        "x": x,
        "y": y, 
        "z": z,
        "roll": roll,
        "pitch": pitch,
        "yaw": yaw
    }
    
    try:
        with open('/tmp/target_position.json', 'w') as f:
            json.dump(target, f, indent=2)
        print(f"Testing position: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        return True
    except Exception as e:
        print(f"Error writing to JSON: {e}")
        return False

def explore_workspace():
    """Explore the workspace bounds systematically."""
    
    # Start from your known good position
    base_x, base_y, base_z = 0.4185, 0.0, 0.3999
    
    print("=== XArm7 Workspace Explorer ===")
    print(f"Starting from: x={base_x*1000:.1f}mm, y={base_y*1000:.1f}mm, z={base_z*1000:.1f}mm")
    print("\nTesting different directions...")
    
    # Test X direction (forward/backward)
    print("\n--- X Direction (Forward/Backward) ---")
    x_forward = base_x + 0.1  # 100mm forward
    x_backward = base_x - 0.1  # 100mm backward
    
    print(f"Testing forward: x={x_forward*1000:.1f}mm")
    test_position(x_forward, base_y, base_z)
    input("Press Enter to continue...")
    
    print(f"Testing backward: x={x_backward*1000:.1f}mm")
    test_position(x_backward, base_y, base_z)
    input("Press Enter to continue...")
    
    # Test Y direction (left/right)
    print("\n--- Y Direction (Left/Right) ---")
    y_left = base_y + 0.1  # 100mm left
    y_right = base_y - 0.1  # 100mm right
    
    print(f"Testing left: y={y_left*1000:.1f}mm")
    test_position(base_x, y_left, base_z)
    input("Press Enter to continue...")
    
    print(f"Testing right: y={y_right*1000:.1f}mm")
    test_position(base_x, y_right, base_z)
    input("Press Enter to continue...")
    
    # Test Z direction (up/down)
    print("\n--- Z Direction (Up/Down) ---")
    z_up = base_z + 0.1  # 100mm up
    z_down = base_z - 0.1  # 100mm down
    
    print(f"Testing up: z={z_up*1000:.1f}mm")
    test_position(base_x, base_y, z_up)
    input("Press Enter to continue...")
    
    print(f"Testing down: z={z_down*1000:.1f}mm")
    test_position(base_x, base_y, z_down)
    input("Press Enter to continue...")
    
    # Return to base position
    print("\n--- Returning to base position ---")
    test_position(base_x, base_y, base_z)
    print("Exploration complete!")

def interactive_explorer():
    """Interactive workspace explorer."""
    print("=== Interactive XArm7 Workspace Explorer ===")
    print("Commands:")
    print("  x <value> - Set X position (in meters)")
    print("  y <value> - Set Y position (in meters)")
    print("  z <value> - Set Z position (in meters)")
    print("  r <value> - Set roll (in degrees)")
    print("  p <value> - Set pitch (in degrees)")
    print("  w <value> - Set yaw (in degrees)")
    print("  base - Return to base position")
    print("  quit - Exit")
    
    # Current position
    current = {"x": 0.4185, "y": 0.0, "z": 0.3999, "roll": -25.5, "pitch": 87.7, "yaw": -25.5}
    
    while True:
        try:
            cmd = input("\nCommand: ").strip().split()
            if not cmd:
                continue
                
            if cmd[0] == "quit":
                break
            elif cmd[0] == "base":
                test_position(current["x"], current["y"], current["z"], current["roll"], current["pitch"], current["yaw"])
            elif cmd[0] in ["x", "y", "z"] and len(cmd) == 2:
                try:
                    value = float(cmd[1])
                    if cmd[0] == "x":
                        current["x"] = value
                    elif cmd[0] == "y":
                        current["y"] = value
                    elif cmd[0] == "z":
                        current["z"] = value
                    test_position(current["x"], current["y"], current["z"], current["roll"], current["pitch"], current["yaw"])
                    print(f"Current: x={current['x']*1000:.1f}mm, y={current['y']*1000:.1f}mm, z={current['z']*1000:.1f}mm")
                except ValueError:
                    print("Invalid value")
            elif cmd[0] in ["r", "p", "w"] and len(cmd) == 2:
                try:
                    value = float(cmd[1])
                    if cmd[0] == "r":
                        current["roll"] = value
                    elif cmd[0] == "p":
                        current["pitch"] = value
                    elif cmd[0] == "w":
                        current["yaw"] = value
                    test_position(current["x"], current["y"], current["z"], current["roll"], current["pitch"], current["yaw"])
                    print(f"Current: roll={current['roll']:.1f}°, pitch={current['pitch']:.1f}°, yaw={current['yaw']:.1f}°")
                except ValueError:
                    print("Invalid value")
            else:
                print("Invalid command")
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")
    
    print("Explorer closed.")

if __name__ == "__main__":
    print("Choose exploration mode:")
    print("1. Systematic exploration")
    print("2. Interactive exploration")
    
    try:
        choice = input("Enter choice (1 or 2): ").strip()
        if choice == "1":
            explore_workspace()
        elif choice == "2":
            interactive_explorer()
        else:
            print("Invalid choice")
    except KeyboardInterrupt:
        print("\nExplorer cancelled.")

#!/usr/bin/env python3
"""XArm7 Position Looper - Smoothly loop between multiple Cartesian positions.

This script:
1. Defines multiple target positions in mm and degrees (easy to copy from UFactory Studio)
2. Smoothly moves between positions with configurable dwell time
3. Automatically converts mm to meters and degrees to radians
4. Easy to add more positions by editing the POSITIONS list

Usage:
1. Edit the POSITIONS list below with your desired positions
2. Run this script while xarm7_self_collision_socket.py is running
3. Watch the robot smoothly move between positions

Format for each position:
{
    "name": "Position Name",           # Human-readable name
    "x": 418.5,                       # X position in mm
    "y": 0.0,                         # Y position in mm  
    "z": 399.9,                       # Z position in mm
    "roll": -25.5,                    # Roll in degrees
    "pitch": 87.7,                    # Pitch in degrees
    "yaw": -25.5,                     # Yaw in degrees
    "dwell_time": 1.0                 # Time to wait at this position (seconds)
}
"""

import json
import time
import math

# ============================================================================
# CONFIGURATION - Edit these positions as needed
# ============================================================================
# 
# Format: [x, y, z, roll, pitch, yaw] where:
# - x, y, z are in millimeters (mm)
# - roll, pitch, yaw are in degrees
# 
# Just copy-paste from UFactory Studio like: [729.5,-18.6,428,4.5,81.6,2.9]

POSITIONS = [
    {
        "name": "Forward Position", 
        "coords": [733.8,11.9,421.2,22.8,88.3,23.9],  # [x, y, z, roll, pitch, yaw]
        "dwell_time": 8.0     # seconds
    },
    {
        "name": "Home Position",
        "coords": [418.5, 0.0, 399.9, -25.5, 87.7, -25.5],  # [x, y, z, roll, pitch, yaw]
        "dwell_time": 8.0     # seconds
    },
    {
        "name": "Left Position",
        "coords": [-27.6,733.4,421.2,22.8,88.3,115.2],  # [x, y, z, roll, pitch, yaw]
        "dwell_time": 8.0     # seconds
    },
    {
        "name": "Home Position",
        "coords": [418.5, 0.0, 399.9, -25.5, 87.7, -25.5],  # [x, y, z, roll, pitch, yaw]
        "dwell_time": 8.0     # seconds
    },
    {
        "name": "Up Position",
        "coords": [-103.9,-2.8,999.2,0,1,1],  # [x, y, z, roll, pitch, yaw]
        "dwell_time": 8.0     # seconds
    },
    {
        "name": "Home Position",
        "coords": [418.5, 0.0, 399.9, -25.5, 87.7, -25.5],  # [x, y, z, roll, pitch, yaw]
        "dwell_time": 8.0     # seconds
    },
    {
        "name": "Right Position",
        "coords": [43.2,-735.2,386.4,87.9,89.4,1.4],  # [x, y, z, roll, pitch, yaw]
        "dwell_time": 8.0     # seconds
    },
    {
        "name": "Home Position",
        "coords": [418.5, 0.0, 399.9, -25.5, 87.7, -25.5],  # [x, y, z, roll, pitch, yaw]
        "dwell_time": 8.0     # seconds
    }
]

# ============================================================================
# SCRIPT SETTINGS
# ============================================================================

# How often to update the position (Hz)
UPDATE_RATE = 2.0  # 10 times per second

# Whether to loop continuously or run once
LOOP_CONTINUOUSLY = True

# ============================================================================
# MAIN SCRIPT
# ============================================================================

def mm_to_meters(mm):
    """Convert millimeters to meters."""
    return mm / 1000.0

def degrees_to_radians(degrees):
    """Convert degrees to radians."""
    return degrees * math.pi / 180.0

def convert_position_to_meters(position):
    """Convert a position from mm/degrees to meters/radians."""
    coords = position["coords"]  # [x, y, z, roll, pitch, yaw]
    return {
        "x": mm_to_meters(coords[0]),      # x in mm -> m
        "y": mm_to_meters(coords[1]),      # y in mm -> m
        "z": mm_to_meters(coords[2]),      # z in mm -> m
        "roll": coords[3],                 # roll in degrees
        "pitch": coords[4],                # pitch in degrees
        "yaw": coords[5]                   # yaw in degrees
    }

def write_position_to_json(position_meters, position_name):
    """Write the position to the targets JSON file that xarm7_self_collision_socket.py reads."""
    try:
        with open('/tmp/target_positions.json', 'w') as f:
            json.dump(position_meters, f, indent=2)
        print(f"✓ Set target: {position_name}")
        print(f"  Target: x={position_meters['x']*1000:.1f}mm, y={position_meters['y']*1000:.1f}mm, z={position_meters['z']*1000:.1f}mm")
        print(f"  Orientation: roll={position_meters['roll']:.1f}°, pitch={position_meters['pitch']:.1f}°, yaw={position_meters['yaw']:.1f}°")
        return True
    except Exception as e:
        print(f"✗ Error writing target: {e}")
        return False

def main():
    """Main loop that moves between positions."""
    print("=== XArm7 Position Looper ===")
    print(f"Total positions: {len(POSITIONS)}")
    print(f"Update rate: {UPDATE_RATE} Hz")
    print(f"Loop continuously: {LOOP_CONTINUOUSLY}")
    print()
    
    # Convert all positions to meters for internal use
    positions_meters = [convert_position_to_meters(pos) for pos in POSITIONS]
    
    # Calculate timing
    update_interval = 1.0 / UPDATE_RATE
    
    position_index = 0
    cycle_count = 0
    
    try:
        while True:
            current_position = POSITIONS[position_index]
            current_position_meters = positions_meters[position_index]
            
            print(f"\n--- Cycle {cycle_count + 1}, Position {position_index + 1}/{len(POSITIONS)} ---")
            print(f"Moving to: {current_position['name']}")
            
            # Write position to JSON file
            if write_position_to_json(current_position_meters, current_position['name']):
                # Wait at this position
                dwell_time = current_position['dwell_time']
                print(f"Waiting {dwell_time} seconds...")
                
                # Update position multiple times during dwell to ensure smooth movement
                updates_needed = int(dwell_time * UPDATE_RATE)
                for i in range(updates_needed):
                    time.sleep(update_interval)
                    # Re-write position to ensure it's maintained
                    write_position_to_json(current_position_meters, current_position['name'])
                
                print(f"✓ Completed dwell at {current_position['name']}")
            else:
                print(f"✗ Failed to move to {current_position['name']}")
                time.sleep(1.0)  # Wait a bit before trying next position
            
            # Move to next position
            position_index = (position_index + 1) % len(POSITIONS)
            
            # If we've completed a full cycle
            if position_index == 0:
                cycle_count += 1
                print(f"\n🎉 Completed cycle {cycle_count}")
                
                if not LOOP_CONTINUOUSLY:
                    print("Script completed (not looping continuously)")
                    break
                else:
                    print("Starting next cycle...")
                    time.sleep(0.5)  # Brief pause between cycles
            
    except KeyboardInterrupt:
        print("\n\n⏹️  Position looper stopped by user")
        print("Robot will stay at current position")
    except Exception as e:
        print(f"\n\n❌ Error in position looper: {e}")
        print("Robot will stay at current position")

if __name__ == "__main__":
    main()

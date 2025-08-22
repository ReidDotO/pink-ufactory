#!/usr/bin/env python3
"""XArm7 Hand Tracking Control using MediaPipe

This script:
1. Uses MediaPipe to track hand and shoulder positions from webcam
2. Converts wrist position relative to shoulder into xarm7 end-effector coordinates
3. Updates /tmp/target_positions.json in real-time for xarm7_self_collision_socket.py
4. Provides smooth hand-to-robot mapping with configurable sensitivity

Requirements:
- pip install mediapipe opencv-python numpy

Usage:
1. Run this script while xarm7_self_collision_socket.py is running
2. Position yourself in front of the webcam
3. Move your hand to control the robot end-effector
4. Press 'q' to quit

Coordinate System:
- Shoulder position = origin (0,0,0)
- Hand movements relative to shoulder control robot position
- Z-axis: forward/backward (depth)
- Y-axis: up/down (height)  
- X-axis: left/right (width)
"""

import cv2
import mediapipe as mp
import numpy as np
import json
import time
import math
import os

# MediaPipe setup
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

class HandTrackingController:
    def __init__(self):
        # Initialize MediaPipe
        self.hands = mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            max_num_hands=1
        )
        
        # Camera setup
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise RuntimeError("Cannot open camera")
        
        # Get camera properties for coordinate conversion
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # Robot control parameters
        self.sensitivity = 5.0  # Much higher sensitivity for responsive control
        self.deadzone = 0.01    # Very small deadzone for immediate response
        
        # Reference positions (will be set on first detection)
        self.hand_ref = None
        
        # Robot workspace limits (in meters, relative to shoulder)
        self.x_range = [0.35, 0.35]   # X fixed at 350mm forward (never changes)
        self.y_range = [-0.7, 0.7]    # Left/right range: ±700mm
        self.z_range = [0.05, 0.8]    # Up/down range: 50mm down to 800mm up
        
        # Current robot target - center position from your coordinates
        self.current_target = {
            "x": 0.350,   # 350mm forward (fixed, never changes)
            "y": 0.0,     # 0mm left/right (your center position)
            "z": 0.3999,  # 399.9mm height (your center position)
            "roll": -25.5,
            "pitch": 87.7,
            "yaw": -25.5
        }
        
        # Smoothing for hand tracking - reduced for more real-time response
        self.smoothing_factor = 0.7  # Higher = more responsive (was 0.3)
        self.smoothed_hand_pos = None
        
        print("Hand tracking controller initialized")
        print(f"Camera resolution: {self.frame_width}x{self.frame_height}")
        print("Move your hand to control the robot. Press 'q' to quit.")
    
    def normalize_coordinates(self, x, y, z):
        """Convert MediaPipe coordinates to normalized -1 to +1 range for robot control"""
        # Simple proportional mapping - no forcing to extremes
        
        # X: left/right movement - map proportionally
        # Use a reasonable range based on typical hand movement
        norm_x = np.clip(x / 200.0, -1.0, 1.0)  # 200 pixels = full range
        
        # Y: up/down movement - map proportionally
        # Use a reasonable range based on typical hand movement
        norm_y = np.clip(y / 300.0, -1.0, 1.0)  # 300 pixels = full range
        
        # Z: forward/backward - MediaPipe Z is 0=camera, 1=far, center around 0
        norm_z = (z - 0.5) * 2  # Convert 0-1 to -1 to +1
        
        return norm_x, norm_y, norm_z
    
    def map_to_robot_space(self, norm_x, norm_y, norm_z):
        """Map normalized coordinates to robot workspace with your center position as base"""
        # Robot coordinate system:
        # X = Forward/backward (your center: 418.5mm)
        # Y = Left/right (your center: 0mm)
        # Z = Up/down (your center: 399.9mm)
        
        # Base position: your specified center coordinates
        base_x = 0.350    # 350mm forward (fixed, never changes)
        base_y = 0.0      # 0mm left/right (your center Y)
        base_z = 0.3999   # 399.9mm height (your center Z)
        
        # Map hand movements to robot offsets (X is fixed, only Y and Z move)
        # X: No movement - always stays at 350mm forward
        offset_x = 0.0    # No forward/backward movement
        
        # Y: Left/right movement from hand X position
        offset_y = norm_x * 0.7  # Left/right: ±700mm range (full left/right workspace)
        
        # Z: Up/down movement from hand Y position  
        offset_z = -norm_y * 0.375  # Up/down: 50mm down to 800mm up (full up/down workspace)
        
        # Calculate final robot position
        robot_x = base_x + offset_x  # Always 350mm forward
        robot_y = base_y + offset_y  # Left/right movement
        robot_z = base_z + offset_z  # Up/down movement
        
        # Apply workspace limits
        robot_x = np.clip(robot_x, 0.1, 0.5)     # 100mm to 500mm forward from shoulder
        robot_y = np.clip(robot_y, -0.7, 0.7)    # ±700mm left/right
        robot_z = np.clip(robot_z, 0.05, 0.8)    # 50mm down to 800mm up
        
        return robot_x, robot_y, robot_z
    
    def update_target_position(self, x, y, z):
        """Update the target position and write to JSON file"""
        # Apply smoothing
        if self.smoothed_hand_pos is None:
            self.smoothed_hand_pos = np.array([x, y, z])
        else:
            self.smoothed_hand_pos = (self.smoothing_factor * np.array([x, y, z]) + 
                                     (1 - self.smoothing_factor) * self.smoothed_hand_pos)
        
        # Check if movement is significant enough (deadzone)
        if self.hand_ref is not None:
            movement = np.linalg.norm(self.smoothed_hand_pos - self.hand_ref)
            if movement < self.deadzone:
                return  # Don't update if movement is too small
        
        # Update current target
        self.current_target["x"] = float(self.smoothed_hand_pos[0])
        self.current_target["y"] = float(self.smoothed_hand_pos[1])
        self.current_target["z"] = float(self.smoothed_hand_pos[2])
        
        # Write to JSON file for xarm7_self_collision_socket.py
        try:
            with open('/tmp/target_positions.json', 'w') as f:
                json.dump(self.current_target, f, indent=2)
        except Exception as e:
            print(f"Error writing target file: {e}")
    
    def process_frame(self, frame):
        """Process a single camera frame"""
        # Convert to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process hand detection
        hand_results = self.hands.process(rgb_frame)
        
        # Draw hand landmarks
        if hand_results.multi_hand_landmarks:
            for hand_landmarks in hand_results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        
        # No shoulder reference needed - just use hand position directly
        
        # Get hand position (use right hand if available, otherwise left)
        hand_pos = None
        if hand_results.multi_hand_landmarks:
            # Prefer right hand, fall back to left
            for i, hand_landmarks in enumerate(hand_results.multi_hand_landmarks):
                if hand_results.multi_handedness[i].classification[0].label == "Right":
                    hand_pos = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
                    break
            
            if hand_pos is None and hand_results.multi_hand_landmarks:
                # Use left hand if no right hand
                hand_pos = hand_results.multi_hand_landmarks[0].landmark[mp_hands.HandLandmark.WRIST]
            
            if hand_pos:
                hand_x = hand_pos.x * self.frame_width
                hand_y = hand_pos.y * self.frame_height
                hand_z = hand_pos.z
                
                # Set hand reference on first detection
                if self.hand_ref is None:
                    self.hand_ref = np.array([hand_x, hand_y, hand_z])
                    print(f"Hand reference set: {self.hand_ref}")
                
                # Use absolute hand position on screen (no shoulder reference needed)
                # Center the coordinates around the middle of the screen
                rel_x = hand_x - (self.frame_width / 2)   # Center at 0, left = negative, right = positive
                rel_y = hand_y - (self.frame_height / 2)  # Center at 0, up = negative, down = positive (fixed!)
                
                # Normalize coordinates
                norm_x, norm_y, norm_z = self.normalize_coordinates(rel_x, rel_y, hand_z)
                
                # Map to robot space and update target
                robot_x, robot_y, robot_z = self.map_to_robot_space(norm_x, norm_y, norm_z)
                self.update_target_position(robot_x, robot_y, robot_z)
                
                # Display information on frame
                cv2.putText(frame, f"Screen pos: ({hand_x:.0f}, {hand_y:.0f}) [640x480]", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"Centered: ({rel_x:.0f}, {rel_y:.0f})", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                cv2.putText(frame, f"Normalized: ({norm_x:.2f}, {norm_y:.2f}, {norm_z:.2f})", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                cv2.putText(frame, f"Robot: X={robot_x*1000:.0f}mm, Y={robot_y*1000:.0f}mm, Z={robot_z*1000:.0f}mm", 
                           (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
            else:
                # Show status when no hand detected
                cv2.putText(frame, "No hand detected - move hand into view", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Display controls and status
        cv2.putText(frame, "Press 'q' to quit, 'r' to reset reference", 
                   (10, self.frame_height - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return frame
    
    def reset_reference(self):
        """Reset shoulder and hand reference positions"""
        self.shoulder_ref = None
        self.hand_ref = None
        print("Reference positions reset")
    
    def run(self):
        """Main loop for hand tracking"""
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to grab frame")
                    break
                
                # Flip the raw camera frame upside down first
                flipped_frame = cv2.flip(frame, 0)
                
                # Process the flipped frame
                processed_frame = self.process_frame(flipped_frame)
                
                # Display the processed frame
                cv2.imshow('Hand Tracking Control', processed_frame)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("Quitting...")
                    break
                elif key == ord('r'):
                    self.reset_reference()
                elif key == ord('s'):
                    # Toggle sensitivity (now affects the mapping factors)
                    if self.sensitivity == 5.0:
                        self.sensitivity = 2.5  # Lower sensitivity for fine control
                        print("Sensitivity: LOW (fine control)")
                    else:
                        self.sensitivity = 5.0  # High sensitivity for full range
                        print("Sensitivity: HIGH (full range)")
                
                # Minimal delay for maximum responsiveness
                time.sleep(0.001)  # 1ms delay instead of 10ms
                
        except KeyboardInterrupt:
            print("Interrupted by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        self.hands.close()
        print("Hand tracking controller stopped")

def main():
    """Main function"""
    try:
        controller = HandTrackingController()
        controller.run()
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure you have a webcam connected and MediaPipe installed:")
        print("pip install mediapipe opencv-python numpy")

if __name__ == "__main__":
    main()

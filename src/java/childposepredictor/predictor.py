import cv2
import numpy as np
import time
import socket
import ntcore

class VisionSystem:
    
    def __init__(self):
        
        self.inst = ntcore.NetworkTableInstance.getDefault()
        # Physical Constants
        self.SQUARE_SIZE = 0.5  # Meters (4 ping pong balls in a square)
        self.BASELINE = 0.5     # Meters (Distance between cameras)
        self.FOCAL_LENGTH = 700 # Pixel constant (Adjust based on camera calibration)
        
        # Tracking history for prediction
        self.history = [] # Stores (timestamp, x, y, z)
        
        # Color Ranges (HSV) - Adjust these for your environment!
        self.BALL_LOW = np.array([20, 100, 100])
        self.BALL_HIGH = np.array([30, 255, 255])
        self.TARGET_LOW = np.array([0, 150, 100])
        self.TARGET_HIGH = np.array([10, 255, 255])
        
    def get_labeled_corners(balls):
        if len(balls) < 4: return None
        
        # 1. Sort all 4 balls by their Y-coordinate (Vertical position in image)
        # The two with the smallest Y are the "Front" row (further away/higher in frame)
        # The two with the largest Y are the "Back" row (closer to camera/lower in frame)
        balls.sort(key=lambda p: p[1])
        
        front_pair = balls[:2]
        back_pair = balls[2:]
        
        # 2. Sort each pair by X-coordinate to find Left vs Right
        front_pair.sort(key=lambda p: p[0])
        back_pair.sort(key=lambda p: p[0])
        
        return {
            "FL": front_pair[0],
            "FR": front_pair[1],
            "BL": back_pair[0],
            "BR": back_pair[1]
        }

    def find_blobs(self, frame, low, high):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, low, high)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        centers = []
        for cnt in contours:
            if cv2.contourArea(cnt) > 100:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centers.append((cx, cy))
        return centers
    
    def find_precise_balls(frame, low, high):
        # 1. Color Masking
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, low, high)
        
        # Clean up the mask (remove tiny specs)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # 2. Shape Detection (Circularly check)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        verified_balls = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            perimeter = cv2.arcLength(cnt, True)
            
            if perimeter == 0: continue
            
            # Circularity Formula: 4 * pi * (Area / Perimeter^2)
            # A perfect circle is 1.0. A square is ~0.78.
            circularity = 4 * np.pi * (area / (perimeter * perimeter))
            
            if 0.7 < circularity < 1.2 and area > 100:
                # Get the center and radius for the square logic
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                verified_balls.append((int(x), int(y)))
                
        return verified_balls

    def calculate_360_yaw(corners):
        # Use the vector from the Back-Left to the Front-Left 
        # This represents the "Forward" pointing arrow of the robot
        dx = corners["FL"][0] - corners["BL"][0]
        dy = corners["FL"][1] - corners["BL"][1]

        # atan2(y, x) returns the angle in radians from -PI to PI
        # It correctly distinguishes between all 4 quadrants
        yaw_rad = np.arctan2(dy, dx)

        return np.degrees(yaw_rad)

    def get_stereo_position(self, xL, xR, y):
        disparity = abs(xL - xR)
        if disparity < 1: return None
        
        z = (self.BASELINE * self.FOCAL_LENGTH) / disparity
        x = (xL * z) / self.FOCAL_LENGTH
        y_val = (y * z) / self.FOCAL_LENGTH
        return np.array([x, y_val, z])

    def predict_future(self, current_pos, lookahead=0.1):
        self.history.append((time.time(), current_pos))
        if len(self.history) > 10: self.history.pop(0)
        
        if len(self.history) < 2: return current_pos
        
        # Velocity = DeltaPos / DeltaTime
        dt = self.history[-1][0] - self.history[0][0]
        dp = self.history[-1][1] - self.history[0][1]
        velocity = dp / dt
        
        return current_pos + (velocity * lookahead)

    def run(self):
        # Init Cameras
        capL = cv2.VideoCapture(0)
        capR = cv2.VideoCapture(1)

        while True:
            retL, frameL = capL.read()
            retR, frameR = capR.read()
            if not retL or not retR: break

            # 1. Find Child (Target)
            childL = self.find_precise_balls(frameL, self.TARGET_LOW, self.TARGET_HIGH)
            childR = self.find_precise_balls(frameR, self.TARGET_LOW, self.TARGET_HIGH)

            # 2. Find Robot Orientation Square
            robot_balls = self.find_precise_balls(frameL, self.BALL_LOW, self.BALL_HIGH)

            if childL and childR and len(robot_balls) >= 4:
                # Calculate Orientation
                yaw = self.calculate_360_yaw(self.get_corner_positions(robot_balls))
                
                # Calculate Stereo 3D
                rel_pos = self.get_stereo_position(childL[0][0], childR[0][0], childL[0][1])
                
                if rel_pos is not None:
                    # Transform based on robot angle
                    # Rotation matrix for Y-axis (Yaw)
                    rot_matrix = np.array([
                        [np.cos(yaw), 0, np.sin(yaw)],
                        [0, 1, 0],
                        [-np.sin(yaw), 0, np.cos(yaw)]
                    ])
                    world_pos = rot_matrix @ rel_pos
                    
                    # Predict
                    future_pos = self.predict_future(world_pos)
                    ## PUBLISH THE DATA TO NT
                    self.inst.getTable("CHILD_POSE_PREDICTOR/HAS_TARGET").putBoolean(True)
                    self.inst.getTable("CHILD_POSE_PREDICTOR/OUTPUT").putNumberArray("future_pos", future_pos)
                    
                    print(f"Robot Yaw: {np.degrees(yaw):.2f}° | Aiming at: {future_pos}")
                else:
                    self.inst.getTable("CHILD_POSE_PREDICTOR/HAS_TARGET").putBoolean(False)
            else:
                self.inst.getTable("CHILD_POSE_PREDICTOR/HAS_TARGET").putBoolean(False)

            # Optional: Show video
            cv2.imshow("Left Camera", frameL)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

        capL.release()
        capR.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    system = VisionSystem()
    system.run()
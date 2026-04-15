import cv2
import numpy as np
import time
import ntcore

class VisionSystem:
    def __init__(self):
        # Initialize NetworkTables
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startClient4("VisionSystem")
        # Try RIO first, fallback to local
        self.inst.setServerTeam(3173) # Or use setServer("10.31.73.2")
        self.table = self.inst.getTable("CHILD_POSE_PREDICTOR")

        self.SQUARE_SIZE = 0.5  
        self.BASELINE = 0.5     
        self.FOCAL_LENGTH = 700 
        self.history = [] 

        self.BALL_LOW = np.array([20, 100, 100])
        self.BALL_HIGH = np.array([30, 255, 255])
        self.TARGET_LOW = np.array([0, 150, 100])
        self.TARGET_HIGH = np.array([10, 255, 255])

    def get_labeled_corners(self, balls):
        if len(balls) < 4: return None
        # Sort by Y (Vertical)
        balls.sort(key=lambda p: p[1])
        front_pair = sorted(balls[:2], key=lambda p: p[0])
        back_pair = sorted(balls[2:], key=lambda p: p[0])
        
        return {
            "FL": front_pair[0], "FR": front_pair[1],
            "BL": back_pair[0], "BR": back_pair[1]
        }

    def find_precise_balls(self, frame, low, high):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, low, high)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        verified_balls = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0: continue
            circularity = 4 * np.pi * (area / (perimeter * perimeter))
            
            if 0.7 < circularity < 1.2 and area > 100:
                (x, y), _ = cv2.minEnclosingCircle(cnt)
                verified_balls.append((int(x), int(y)))
        return verified_balls

    def calculate_360_yaw(self, corners):
        # Flip Y subtraction because OpenCV Y increases DOWNWARDS
        dx = corners["FR"][0] - corners["FL"][0]
        dy = corners["FL"][1] - corners["FR"][1] 
        return np.arctan2(dy, dx)

    def get_stereo_position(self, xL, xR, y):
        disparity = abs(xL - xR)
        if disparity < 1: return None
        z = (self.BASELINE * self.FOCAL_LENGTH) / disparity
        x = ((xL - 320) * z) / self.FOCAL_LENGTH # Centering X (assuming 640 width)
        y_val = ((y - 240) * z) / self.FOCAL_LENGTH # Centering Y
        return np.array([x, y_val, z])

    def run(self):
        capL = cv2.VideoCapture(0)
        capR = cv2.VideoCapture(1)

        while True:
            retL, frameL = capL.read()
            retR, frameR = capR.read()
            if not retL or not retR: break

            childL = self.find_precise_balls(frameL, self.TARGET_LOW, self.TARGET_HIGH)
            childR = self.find_precise_balls(frameR, self.TARGET_LOW, self.TARGET_HIGH)
            robot_balls = self.find_precise_balls(frameL, self.BALL_LOW, self.BALL_HIGH)

            if childL and childR and len(robot_balls) >= 4:
                corners = self.get_labeled_corners(robot_balls)
                yaw = self.calculate_360_yaw(corners)
                rel_pos = self.get_stereo_position(childL[0][0], childR[0][0], childL[0][1])

                if rel_pos is not None:
                    # Yaw Rotation Matrix
                    s, c = np.sin(yaw), np.cos(yaw)
                    rot_matrix = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
                    world_pos = rot_matrix @ rel_pos
                    
                    self.table.getEntry("HAS_TARGET").setBoolean(True)
                    self.table.getEntry("future_pos").setDoubleArray(world_pos.tolist())
                    print(f"Yaw: {np.degrees(yaw):.1f} World Pos: {world_pos}")
            else:
                self.table.getEntry("HAS_TARGET").setBoolean(False)

            cv2.imshow("Debug", frameL)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

        capL.release()
        capR.release()
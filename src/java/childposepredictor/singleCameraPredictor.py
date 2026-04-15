import cv2
import numpy as np
import time
import ntcore

class SingleCameraVision:
    def __init__(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startClient4("SingleCamVision")
        self.inst.setServerTeam(3173) 
        self.table = self.inst.getTable("CHILD_POSE_PREDICTOR")

        # Physical Constants
        self.REAL_SQUARE_WIDTH = 0.5  # Meters
        self.FOCAL_LENGTH = 700       # Adjust this via calibration!
        
        self.history = [] 

        # Colors
        self.BALL_LOW = np.array([20, 100, 100])
        self.BALL_HIGH = np.array([30, 255, 255])
        self.TARGET_LOW = np.array([0, 150, 100])
        self.TARGET_HIGH = np.array([10, 255, 255])

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

    def get_labeled_corners(self, balls):
        if len(balls) < 4: return None
        balls.sort(key=lambda p: p[1]) # Sort Y
        front_pair = sorted(balls[:2], key=lambda p: p[0]) # Sort X
        back_pair = sorted(balls[2:], key=lambda p: p[0])
        return {"FL": front_pair[0], "FR": front_pair[1], "BL": back_pair[0], "BR": back_pair[1]}

    def calculate_pose(self, corners):
        # 1. Calculate Yaw (Angle)
        dx_yaw = corners["FR"][0] - corners["FL"][0]
        dy_yaw = corners["FL"][1] - corners["FR"][1] # Correct for inverted Y
        yaw = np.arctan2(dy_yaw, dx_yaw)

        # 2. Calculate Distance (Z) based on pixel width of the square
        pixel_width = abs(corners["FR"][0] - corners["FL"][0])
        if pixel_width < 1: return None, None
        
        z = (self.REAL_SQUARE_WIDTH * self.FOCAL_LENGTH) / pixel_width
        
        # 3. Calculate X relative to camera center (assuming 640 width)
        center_x = (corners["FL"][0] + corners["FR"][0]) / 2
        x = ((center_x - 320) * z) / self.FOCAL_LENGTH
        
        return np.array([x, 0, z]), yaw

    def run(self):
        cap = cv2.VideoCapture(0) # Only one camera needed

        while True:
            ret, frame = cap.read()
            if not ret: break

            robot_balls = self.find_precise_balls(frame, self.BALL_LOW, self.BALL_HIGH)
            child_blobs = self.find_precise_balls(frame, self.TARGET_LOW, self.TARGET_HIGH)

            if len(robot_balls) >= 4:
                corners = self.get_labeled_corners(robot_balls)
                rel_pos, yaw = self.calculate_pose(corners)

                if rel_pos is not None:
                    # Log to NetworkTables
                    self.table.getEntry("RobotYaw").setDouble(np.degrees(yaw))
                    self.table.getEntry("RobotPos").setDoubleArray(rel_pos.tolist())
                    
                    # If we see the child too, we can find their relative position
                    if child_blobs:
                        # Estimate child distance based on Y position (Ground Plane Assumption)
                        # Higher in frame = further away
                        print(f"Robot at {rel_pos} Yaw: {np.degrees(yaw):.1f}")

            cv2.imshow("Single Cam Debug", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    SingleCameraVision().run()
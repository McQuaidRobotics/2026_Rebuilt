import cv2
import numpy as np

def nothing(x):
    pass

# 1. Create a Window for the GUI
cv2.namedWindow("Circle Tracker")

# 2. Create Trackbars for HSV tuning (Defaulting to a wide range)
cv2.createTrackbar("Low H", "Circle Tracker", 0, 179, nothing)
cv2.createTrackbar("Low S", "Circle Tracker", 100, 255, nothing)
cv2.createTrackbar("Low V", "Circle Tracker", 100, 255, nothing)
cv2.createTrackbar("High H", "Circle Tracker", 179, 179, nothing)
cv2.createTrackbar("High S", "Circle Tracker", 255, 255, nothing)
cv2.createTrackbar("High V", "Circle Tracker", 255, 255, nothing)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret: break

    # Get current trackbar positions
    l_h = cv2.getTrackbarPos("Low H", "Circle Tracker")
    l_s = cv2.getTrackbarPos("Low S", "Circle Tracker")
    l_v = cv2.getTrackbarPos("Low V", "Circle Tracker")
    h_h = cv2.getTrackbarPos("High H", "Circle Tracker")
    h_s = cv2.getTrackbarPos("High S", "Circle Tracker")
    h_v = cv2.getTrackbarPos("High V", "Circle Tracker")

    # 3. Process Image
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_limit = np.array([l_h, l_s, l_v])
    upper_limit = np.array([h_h, h_s, h_v])
    
    mask = cv2.inRange(hsv, lower_limit, upper_limit)
    mask = cv2.medianBlur(mask, 5) # Reduce noise

    # 4. Find and Filter Shapes
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 500: continue # Ignore tiny specs
        
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0: continue
        
        # Circularity check
        circularity = 4 * np.pi * (area / (perimeter * perimeter))
        
        if 0.7 < circularity < 1.2:
            # Calculate Center
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            center = (int(x), int(y))
            
            # Calculate coordinates relative to center of screen (assuming 640x480)
            rel_x = int(x - 2560.0/2.0)
            rel_y = int(800 - y) # Flip Y so 'up' is positive
            
            # 5. Draw the GUI elements
            cv2.circle(frame, center, int(radius), (0, 255, 0), 2) # Green Circle
            cv2.drawMarker(frame, center, (255, 0, 0), cv2.MARKER_CROSS, 15, 2) # Blue Cross
            
            # Display coordinates on the screen
            text = f"X: {rel_x}, Y: {rel_y}"
            cv2.putText(frame, text, (center[0] + 10, center[1] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Show the results
    cv2.imshow("Circle Tracker", frame)
    cv2.imshow("Mask (What the Robot Sees)", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
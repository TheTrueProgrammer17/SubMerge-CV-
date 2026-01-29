import cv2
import numpy as np
import time

# ------------------ GStreamer Pipeline ------------------
GST_PIPELINE = (
    "v4l2src device=/dev/video0 ! "
    "video/x-raw,width=640,height=480,framerate=30/1 ! "
    "videoconvert ! appsink drop=1"
)

cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open camera")
    exit()

# ------------------ Parameters ------------------
AREA_THRESHOLD = 3000
ALIGNMENT_THRESHOLD = 30
BALLOON_DIAMETER_M = 0.25
MIN_CIRCULARITY = 0.7

# HSV ranges
COLOR_RANGES = {
    "RED": [
        (np.array([0, 120, 70]), np.array([10, 255, 255])),
        (np.array([170, 120, 70]), np.array([180, 255, 255]))
    ],
    "BLUE": [(np.array([100, 150, 50]), np.array([140, 255, 255]))],
    "GREEN": [(np.array([35, 100, 50]), np.array([85, 255, 255]))],
    "YELLOW": [(np.array([20, 120, 120]), np.array([35, 255, 255]))]
}

def is_round(cnt):
    area = cv2.contourArea(cnt)
    if area < AREA_THRESHOLD:
        return False
    perimeter = cv2.arcLength(cnt, True)
    if perimeter == 0:
        return False
    circularity = 4 * np.pi * area / (perimeter ** 2)
    return circularity > MIN_CIRCULARITY

# FPS tracking
prev_time = time.time()

# ------------------ Main Loop ------------------
while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w = frame.shape[:2]
    frame_center = (w // 2, h // 2)

    # Draw X and Y axis lines (crosshair)
    cv2.line(frame, (frame_center[0], 0), (frame_center[0], h), (255, 255, 255), 1)
    cv2.line(frame, (0, frame_center[1]), (w, frame_center[1]), (255, 255, 255), 1)

    # Faster blur
    blurred = cv2.medianBlur(frame, 7)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    best_contour = None
    detected_color = "UNKNOWN"
    max_area = 0

    for color_name, ranges in COLOR_RANGES.items():
        mask = None
        for lower, upper in ranges:
            temp = cv2.inRange(hsv, lower, upper)
            mask = temp if mask is None else cv2.bitwise_or(mask, temp)

        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > max_area and is_round(cnt):
                max_area = area
                best_contour = cnt
                detected_color = color_name

    if best_contour is not None:
        (x, y), radius = cv2.minEnclosingCircle(best_contour)
        balloon_center = (int(x), int(y))

        dx_px = balloon_center[0] - frame_center[0]
        dy_px = frame_center[1] - balloon_center[1]

        meters_per_pixel = BALLOON_DIAMETER_M / (2 * radius)
        dx_m = dx_px * meters_per_pixel
        dy_m = dy_px * meters_per_pixel

        cv2.drawContours(frame, [best_contour], -1, (0, 255, 0), 2)
        cv2.circle(frame, balloon_center, int(radius), (255, 0, 0), 2)

        instructions = []
        if abs(dx_px) > ALIGNMENT_THRESHOLD:
            instructions.append(f"MOVE {'RIGHT' if dx_px > 0 else 'LEFT'}")
        if abs(dy_px) > ALIGNMENT_THRESHOLD:
            instructions.append(f"MOVE {'UP' if dy_px < 0 else 'DOWN'}")

        if not instructions:
            instructions.append("ALIGNED --> Push")
            print("PUSH COMMAND TRIGGERED")

        # Show detected color + instructions
        # Vertical text block
        text_x = 20
        text_y = 40
        line_spacing = 40

        cv2.putText(frame, f"{detected_color} BALLOON", (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

        for i, line in enumerate(instructions):
            cv2.putText(frame, line, (text_x, text_y + (i + 1) * line_spacing),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        # Show offset calculator
        offset_lines = [
            f"dx = {dx_px}px ({dx_m:.2f}m)",
            f"dy = {dy_px}px ({dy_m:.2f}m)"
        ]

        offset_x = 20
        offset_y = 140
        offset_spacing = 30

        for i, line in enumerate(offset_lines):
            cv2.putText(frame, line, (offset_x, offset_y + i * offset_spacing),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    else:
        cv2.putText(frame, "Searching...", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # FPS display
    # curr_time = time.time()
    # fps = 1 / (curr_time - prev_time)
    # prev_time = curr_time
    # cv2.putText(frame, f"FPS: {int(fps)}", (20, h - 20),
    #             cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

    cv2.imshow("Balloon Detection - Pi 4", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

import cv2
import numpy as np

# ------------------ GStreamer Pipeline ------------------
GST_PIPELINE = (
    "v4l2src device=/dev/video0 ! "
    "video/x-raw,width=640,height=480,framerate=30/1 ! "
    "videoconvert ! appsink"
)

# Open camera
cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Failed to open camera")
    exit()

# ------------------ Main Loop ------------------
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # -------- Underwater Filters --------
    # Convert to LAB and apply CLAHE for contrast
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l = clahe.apply(l)
    lab = cv2.merge((l, a, b))
    frame = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    # Boost reds (compensate underwater color loss)
    b, g, r = cv2.split(frame)
    r = cv2.addWeighted(r, 1.5, g, -0.2, 0)
    frame = cv2.merge((b, g, r))

    # Gamma correction for brightness
    gamma = 1.2
    lookup = np.array([((i / 255.0) ** (1.0 / gamma)) * 255
                       for i in range(256)]).astype("uint8")
    frame = cv2.LUT(frame, lookup)

    # -------- Display --------
    cv2.imshow("Underwater Camera Feed", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ------------------ Cleanup ------------------
cap.release()
cv2.destroyAllWindows()

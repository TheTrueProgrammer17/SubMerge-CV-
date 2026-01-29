import cv2

# ------------------ GStreamer Pipeline ------------------
GST_PIPELINE = (
    "v4l2src device=/dev/video0 ! "
    "video/x-raw,width=640,height=480,framerate=30/1 ! "
    "videoconvert ! appsink drop=1"
)

# Open camera
cap = cv2.VideoCapture(GST_PIPELINE, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("❌ Failed to open camera")
    exit()

# ------------------ Main Loop ------------------
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    cv2.imshow("Camera Feed", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

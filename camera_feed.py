#!/usr/bin/env python3
import cv2
import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst

Gst.init(None)

def underwater_filter(frame):
    """Apply underwater-specific corrections."""

    # Convert to LAB for CLAHE
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)

    # CLAHE on L channel (contrast enhancement)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    cl = clahe.apply(l)

    lab = cv2.merge((cl, a, b))
    frame = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    # Boost red channel (compensate underwater absorption)
    b, g, r = cv2.split(frame)
    r = cv2.addWeighted(r, 1.5, r, 0, 0)  # amplify reds
    frame = cv2.merge((b, g, r))

    # Gamma correction (brighten dark underwat er scenes)
    gamma = 1.2
    invGamma = 1.0 / gamma
    table = np.array([(i / 255.0) ** invGamma * 255 for i in np.arange(256)]).astype("uint8")
    frame = cv2.LUT(frame, table)

    return frame

def main():
    pipeline_str = (
        "v4l2src ! videoconvert ! "
        "videobalance brightness=0.0 contrast=1.2 saturation=1.0 ! "
        "appsink emit-signals=true sync=false max-buffers=1 drop=true "
        "caps=video/x-raw,format=BGR"
    )

    pipeline = Gst.parse_launch(pipeline_str)
    appsink = pipeline.get_by_name("appsink0")
    pipeline.set_state(Gst.State.PLAYING)

    cv2.namedWindow("Underwater Filtered Feed", cv2.WINDOW_NORMAL)

    while True:
        sample = appsink.emit("pull-sample")
        if sample is None:
            continue

        buf = sample.get_buffer()
        caps = sample.get_caps()
        width = caps.get_structure(0).get_value("width")
        height = caps.get_structure(0).get_value("height")

        success, mapinfo = buf.map(Gst.MapFlags.READ)
        if not success:
            continue

        frame = np.frombuffer(mapinfo.data, dtype=np.uint8).reshape((height, width, 3))
        buf.unmap(mapinfo)

        # Apply underwater filter
        filtered = underwater_filter(frame)

        cv2.imshow("Underwater Filtered Feed", filtered)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    pipeline.set_state(Gst.State.NULL)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

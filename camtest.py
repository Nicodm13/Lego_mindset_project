# This ensures that the webcam loads instantly, otherwise it takes almost a minute...
# ----------------------------------------------------------
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
# ----------------------------------------------------------

import cv2

# Open a connection to the webcam (0 is usually the default camera)
cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()  # Read a frame from the webcam
    if not ret:
        break
    
    cv2.imshow("Webcam Feed", frame)  # Display the frame
    
    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

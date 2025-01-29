from djitellopy import Tello
import cv2
import time

tello = Tello()
tello.connect()

tello.streamon()

while True:
    frame = tello.get_frame_read().frame
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)  # Correction des couleurs

    cv2.imshow("Tello Video Stream", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

tello.streamoff()
cv2.destroyAllWindows()
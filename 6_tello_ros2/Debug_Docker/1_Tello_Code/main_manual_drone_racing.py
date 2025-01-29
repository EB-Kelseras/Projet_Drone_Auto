import time
from djitellopy import Tello
from parameters import ENV, run_status, FPS, DRONE_POS, RAD2DEG
from subsys_display_view import Display
from subsys_read_keyboard import ReadKeyboard
from subsys_markers_detected import MarkersDetected
from subsys_select_target_marker import SelectTargetMarker
from subsys_tello_sensors import TelloSensors
from subsys_tello_actuators import TelloActuators


def setup():
    ENV.status = ENV.REAL
    TelloSensors.setup()
    TelloActuatortttttts.setup(TelloSensors.TELLO)
    Display.setup()
    ReadKeyboard.setup()
    MarkersDetected.setup()
    SelectTargetMarker.setup()


def run():
    

        # Run keyboard subsystem to get command status
    rc_status_1, key_status, mode_status = ReadKeyboard.run(rc_threshold=40)

    # Control drone using the mode status (takeoff, land, emergency, etc.)
    image, status = TelloSensors.run(mode_status)

    # Send control commands to drone based on keyboard input
    TelloActuators.run(rc_status_1)

    # Retrieve frame from Tello's video feed
    frame = TelloSensors.image()

    # Detect markers in the frame
    markers_status, frame = MarkersDetected.run(frame)

    # Run the target marker selection, returning a list of MarkerStatus tobjects
    detected_markers = SelectTargetMarker.run(frame, markers_status, DRONE_POS, offset=(0, 0))

    # Pass the entire list of detected markers to the display
    Display.run(frame, detected_markers)

    time.sleep(1 / FPS)


def stop():
    Display.stop()
    TelloSensors.stop()
    MarkersDetected.stop()
    SelectTargetMarker.stop()


if __name__ == "__main__":
    setup()

    while run_status.value:
        run()

    stop()

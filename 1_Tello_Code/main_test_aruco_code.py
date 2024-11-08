import time

from parameters import run_status, FPS, DRONE_POS, RAD2DEG
from subsys_display_view import Display
from subsys_read_cam import ReadCAM
from subsys_read_cam import ReadCAMTello
from subsys_read_keyboard import ReadKeyboard
from subsys_markers_detected import MarkersDetected
from subsys_select_target_marker import SelectTargetMarker


def setup():

    ReadCAMTello.setup()
    Display.setup()
    ReadKeyboard.setup()
    MarkersDetected.setup()
    SelectTargetMarker.setup()

def run():
    # Run keyboard subsystem
    rc_status_1, key_status, mode_status = ReadKeyboard.run(rc_threshold=40)
    
    frame = ReadCAMTello.run()

    # Detect markers in the frame
    markers_status, frame = MarkersDetected.run(frame)

    # Run the target marker selection, returning a list of MarkerStatus objects
    detected_markers = SelectTargetMarker.run(frame, markers_status, DRONE_POS, offset=(0, 0))

    # Pass the entire list of detected markers to the display
    Display.run(frame, detected_markers)

    time.sleep(1 / FPS)


def stop():
    Display.stop()
    ReadCAMTello.stop()
    MarkersDetected.stop()
    SelectTargetMarker.stop()


if __name__ == "__main__":
    setup()

    while run_status.value:
        run()

    stop()

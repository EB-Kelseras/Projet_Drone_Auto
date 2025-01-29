import time

from parameters import run_status, FPS, DRONE_POS, RAD2DEG
from subsys_display_view import Display
from subsys_read_cam import ReadCAM
from subsys_read_cam import ReadCAMTello
from subsys_read_keyboard import ReadKeyboard
from subsys_markers_detected import MarkersDetected
from subsys_select_target_marker import SelectTargetMarker
import numpy as np
import cv2


def setup():

    ReadCAM.setup()
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
    newcameramtx = np.array([[606.69636746,0.,274.27103909],[0.,704.55521323,215.84623519],[0.,0.,1.]])
    dist = np.array([[-1.16736408e-01,4.22341490e+00,5.01202143e-03,-2.00548162e-03,-2.58251568e+01]])
    rvecs, tvecs, nada = SelectTargetMarker.my_estimatePoseSingleMarkers(markers_status.corners,0.07,newcameramtx,dist)
    position_drone_global = np.zeros((3, 1)) 
    if rvecs != [] and tvecs != []:
        if isinstance(rvecs, list):
            # Si rvecs est une liste, convertissez-la en numpy array
            rvecs = np.array(rvecs, dtype=np.float32)
        if isinstance(tvecs, list):
            # Si tvecs est une liste, convertissez-la en numpy array
            tvecs = np.array(tvecs, dtype=np.float32)

        # Extraction des vecteurs de rotation et de translation de la première estimation
        rvecs = rvecs[0]  # Récupérer le premier vecteur de rotation
        tvecs = tvecs[0]  # Récupérer le premier vecteur de translation

        # Vérification de la forme des vecteurs de rotation et de translation
        if rvecs.shape == (3, 1) and tvecs.shape == (3, 1):
            # Calcul de la matrice de rotation R à partir du vecteur de rotation rvecs
            R, _ = cv2.Rodrigues(rvecs)

            # Calcul de la position du marqueur dans le repère global
            position_camera = -np.dot(R.T, tvecs)  # Position dans le repère global
            position_drone_global = position_camera.T  # Transpose pour obtenir une forme 3x1 correcte

            # Affichage de la position du drone dans le repère global
            print("Position du drone dans le repère global :")
            print(position_drone_global)
        else:
            print("Erreur : rvecs ou tvecs n'ont pas la forme (3, 1).")



    # Pass the entire list of detected markers to the display
    Display.run(frame, detected_markers)

    time.sleep(1 / FPS)


def stop():
    Display.stop()
    ReadCAM.stop()
    MarkersDetected.stop()
    SelectTargetMarker.stop()


if __name__ == "__main__":
    setup()

    while run_status.value:
        run()

    stop()

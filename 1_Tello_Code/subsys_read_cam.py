import cv2
from djitellopy import Tello


class ReadCAM:
    vid = None  # video capture object

    @classmethod
    def setup(cls):
        # ref to use camera with opencv: https://www.geeksforgeeks.org/python-opencv-capture-video-from-camera/
        # define a video capture object
        cls.vid = cv2.VideoCapture(0)

    @classmethod
    def run(cls):
        ret, frame = cls.vid.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return frame

    @classmethod
    def stop(cls):
        cls.vid.release()




class ReadCAMTello:
    tello = None  # Tello drone object

    @classmethod
    def setup(cls):
        # Initialize the Tello object and connect to the drone
        cls.tello = Tello()
        cls.tello.connect()

        # Start the video stream
        cls.tello.streamon()

    @classmethod
    def run(cls):
        # Get the video frame from the Tello
        frame = cls.tello.get_frame_read().frame

        # Optionally, you can resize or process the frame here

        return frame

    @classmethod
    def stop(cls):
        # Turn off the video stream and disconnect from the drone
        cls.tello.streamoff()
        cls.tello.end()
import os
import time
import numpy as np
from PIL import Image
import glob


class BaseCamera:

    def run_threaded(self):
        return self.frame

class CSICamera(BaseCamera):
    def gstreamer_pipeline(self,capture_width=120, capture_height=160, display_width=120, display_height=160, framerate=20, flip_method=0) :   
        return ('nvarguscamerasrc ! ' 
        'video/x-raw(memory:NVMM), '
        'width=(int)%d, height=(int)%d, '
        'format=(string)NV12, framerate=(fraction)%d/1 ! '
        'nvvidconv flip-method=%d ! '
        'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
        'videoconvert ! '
        'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))
    
    def __init__(self, resolution=(120, 160), framerate=60):
        import cv2
        # initialize the camera and stream
        self.camera = cv2.VideoCapture(self.gstreamer_pipeline(display_width=resolution[1],display_height=resolution[0],flip_method=0), cv2.CAP_GSTREAMER)
        self.ret , self.image = self.camera.read()
        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None

        print('CSICamera loaded.. .warming camera')
        time.sleep(2)

    def run(self):
        ret, frame = self.camera.read()
        return frame
    
    def update(self):
        pass

    def shutdown(self):
        # indicate that the thread should be stopped
        print('stoping CSICamera')
        time.sleep(.5)
        del(self.camera)


class PiCamera(BaseCamera):
    def __init__(self, resolution=(120, 160), framerate=20):
        from picamera.array import PiRGBArray
        from picamera import PiCamera
        resolution = (resolution[1], resolution[0])
        # initialize the camera and stream
        self.camera = PiCamera()  # PiCamera gets resolution (height, width)
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,
                                                     format="rgb",
                                                     use_video_port=True)

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.on = True

        print('PiCamera loaded.. .warming camera')
        time.sleep(2)

    def run(self):
        f = next(self.stream)
        frame = f.array
        self.rawCapture.truncate(0)
        return frame

    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = f.array
            self.rawCapture.truncate(0)

            # if the thread indicator variable is set, stop the thread
            if not self.on:
                break

    def shutdown(self):
        # indicate that the thread should be stopped
        self.on = False
        print('stoping PiCamera')
        time.sleep(.5)
        self.stream.close()
        self.rawCapture.close()
        self.camera.close()

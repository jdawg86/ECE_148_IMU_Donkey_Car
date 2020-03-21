import time
import cv2
import numpy as np

class CVCam():
    count = 0
    '''
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.frame = None
    '''
    #def __init__(self, image_w=160, image_h=120, image_d=3, capture_width=3280, capture_height=2464, framerate=60, gstreamer_flip=0):

    def __init__(self):
        import cv2

        # initialize the camera and stream
        self.cap = cv2.VideoCapture(0)
        self.frame = None
        self.on = True
        self.seeShape = False

        print('init CVCam loaded.. .warming camera')
        time.sleep(2)
 
    def update(self):
        while self.on:
            print("updating")
            self.read()
            self.poll_camera()

    def poll_camera(self):
        import cv2
        print("poll")
        frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([110, 110, 56])
        upper_blue = np.array([160, 255, 180])

        mask = cv2.inRange(frame, lower_blue, upper_blue)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        font = cv2.FONT_HERSHEY_COMPLEX

        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]

            if area > 400:
                cv2.drawContours(frame, [approx], 0, (0, 0, 0), 5)

                if len(approx) == 3:
                    cv2.putText(frame, "Triangle", (x, y), font, 1, (0, 0, 0))
                    print("T R I A N G L E"+ "\n")
                    self.seeShape = True
                if len(approx) == 4:
                    # cv2.drawContours(frame, [approx], 0, (0, 180, 255), 5)
                    cv2.putText(frame, "Rectangle", (x, y), font, 1, (0, 0, 0))
                    print("R E C T A N G L E"+ "\n")
                    self.seeShape = True
                else:
                    print("I don't see anything :'("+"\n")

    def read(self):
        print("run")
        try:
            ret, self.frame = self.cap.read()
            self.poll_camera()
        except:
            pass
        
        return self.seeShape
    
    run = read

    def run_threaded(self):
        print("run_threaded")
        return self.seeShape

    def shutdown(self):
        self.cap.release()
        cv2.destroyAllWindows()

if __name__=="__main__":
    print("main")
    cam = CVCam()
    cam.run()


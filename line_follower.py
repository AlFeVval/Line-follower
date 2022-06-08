#!/usr/bin/env python3
import rospy
import time
import cv2
import imutils
import numpy as np
from std_msgs.msg import Float32

""" Definicion de variables globales """
MAX_FPS = 1/3 #1/n lecturas por frame
lastPublication = 0
globalFrame = None

#Gstreamer pipeline settings
def gstreamer_pipeline(
    capture_width=640,
    display_height=480,
    capture_height=480,
    display_width=640,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def region_of_interest(canny):
    height = canny.shape[0]
    width = canny.shape[1]
    mask = np.zeros_like(canny)
    trapezoid = np.array([[
    (width*1/6, height),
    (width*3/9, height*1/2),
    (width*6/9,height*1/2),
    (width*5/6, height),
    ]], np.int32)
    cv2.fillPoly(mask, trapezoid, 255)
    #cv2.fillPoly(mask, trapezoid, 0)
    masked_image = cv2.bitwise_and(canny, mask)
    #cv2.polylines(masked_image, trapezoid, True, 255, 2)
    return masked_image

def process_image(image):
    #Gray conversion to lane_image
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    th = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)[1]
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    morph = cv2.morphologyEx(th, cv2.MORPH_CLOSE, kernel)
    thresh = cv2.adaptiveThreshold(morph,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,3,0)
    return thresh

def getContours(image):
    cnts = cv2.findContours(image.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    return cnts

def setFrame(img):
    globalFrame = img

def startRecord():
    global lastPublication
    global MAX_FPS
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    run = cap.isOpened()
    while(run):
        ret,frame = cap.read()
        if np.abs(time.time()-lastPublication) > MAX_FPS:
            try:
                setFrame(frame)
            except Exception as e:
                print(e)
            lastPublication = time.time()
        #cv2.imshow("Frame",frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            run = False
            break
    cap.release()
    cv2.destroyAllWindows()

global rate
class follower:
    def __init__(self):
        self.giro = 0.0
        #ROS INIT
        self.pub_steering = rospy.Publisher('steering', Float32, queue_size=8)
        self.pub_throttle = rospy.Publisher('throttle', Float32, queue_size=8)
        rospy.init_node('line_follower', anonymous=True)
        self.rate = rospy.Rate(1) # 30hz
        #CONTROL INIT
        self.my_centroid = 0,0
        self.prev_centroid = 0,0
        self.error = 0,0
        self.prev_error = 0,0

    def centroidToTurn(self):
        kp = float(1/75)
        kd = float(1/500)
        self.error = (self.prev_centroid[0] - self.my_centroid[0]),(self.prev_centroid[1] - self.my_centroid[1])
        print("Error: "+str(self.error)+", Prev_error:"+str(self.prev_error))
        if(self.prev_error[0] > self.error[0]):
            print("<==== giro a la izquierda")
            self.giro = kp*float(self.prev_error[0]) + kd*float(self.error[0])
        if(self.prev_error[0] < self.error[0]):
            print("giro a la derecha ====>")
            self.giro = kp*float(self.prev_error[0]) + kd*float(self.error[0])
        if(self.prev_error[0] == self.error[0]):
            print(" ==== En linea  ====")
            self.giro = 0.05
        self.prev_centroid = (self.my_centroid[0],self.my_centroid[1])
        self.prev_error = (self.error[0],self.error[1])

    def stop(self):
        while rospy.is_shutdown():
            throttle = 0.0 
            steering = 0.0
            #Pubblish values
            self.pub_throttle.publish(throttle)
            self.pub_steering.publish(steering)
            self.rate.sleep()

    def movement(self):
        while not rospy.is_shutdown():
            self.centroidToTurn()
            throttle = 0.35 
            steering = self.giro
            #Pubblish values
            self.pub_throttle.publish(throttle)
            self.pub_steering.publish(steering)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        #line_follower init
        startRecord()
        cv2.imshow("Frame",globalFrame)
        #my_follower = follower()
        
    except rospy.ROSInterruptException:
        pass
    

    # cap = cv2.VideoCapture("circuito.mp4")
    # my_follower = follower()
    # while(cap.isOpened()):
    #     ret,frame = cap.read()
    #     filtered = process_image(frame)
    #     cropped = region_of_interest(filtered)
    #     contours = getContours(cropped)
    #     if len(contours) > 0:
    #         # c = max(contours, key = cv2.contourArea)
    #         # print(c)
    #         M = cv2.moments(cropped)
    #         if M["m00"]!=0:
    #             cx = int(M['m10']/M['m00'])
    #             # new_cx = getError(cx)
    #             # error = cx - int(new_cx)
    #             cy = int(M['m01']/M['m00'])
    #             Centroid = cx,cy
    #             my_follower.getCentroid(Centroid[0],Centroid[1])
    #         # print("CX: "+str(cx)+" CY: "+str(cy))
    #         # print("CX: "+str(cx)+" error: "+str(error)+" CY: No me interesa")
    #         cv2.circle(frame,(cx,cy),5,(0,0,255),-1)
    #         # cv2.circle(frame,(error,cy),5,(0,255,0),-1)
    #         # self.printCentroid()
    #     #cv2.drawContours(frame,contours,-1,(0,0,255),1)
    #
    #     if np.abs(time.time()-lastPublication) > MAX_FPS:
    #         try:
    #             my_follower.centroidToTurn()
    #             # my_follower.printCentroid()
    #         except Exception as e:
    #             print(e)
    #         lastPublication = time.time()
    #
    #     cv2.imshow("Frame",frame)
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break
    # cap.release()
    # cv2.destroyAllWindows()

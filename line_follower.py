#!/usr/bin/env python3
import time
import cv2
import imutils
import numpy as np
import rospy
from jetracer.nvidia_racecar import NvidiaRacecar

""" Definicion de variables globales """
MAX_FPS = 1 #1/n
lastPublication = 0

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


class follower:
    def __init__(self):
        #ROS INIT
        self.car = NvidiaRacecar()
        self.car.steering = rospy.Publisher('throttle', Float32, queue_size=8)
        self.car.throttle = rospy.Publisher('steering', Float32, queue_size=8)
        rospy.init_node('line_followe', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        #CONTROL INIT
        self.my_centroid = 0,0
        self.prev_centroid = 0,0
        self.error = 0,0
        self.prev_error = 0,0

    def centroidToTurn(self):
        kd = self.error[0]/30
        self.error = (self.prev_centroid[0] - self.my_centroid[0]),(self.prev_centroid[1] - self.my_centroid[1])
        print("Error: "+str(self.error)+", Prev_error:"+str(self.prev_error))
        if(self.prev_error[0] > self.error[0]):
            print("<==== giro a la izquierda")
            if (kd*-1.0 < -1.0):
                giro = -1.0
            else:
                giro = kd*-1.0
        if(self.prev_error[0] < self.error[0]):
            print("giro a la derecha ====>")
            if (kd*-1.0 > 1.0):
                giro = 1.0
            else:
                giro = kd*-1.0
        if(self.prev_error[0] == self.error[0]):
            print(" ==== En linea  ====")
            giro = 0.05
        self.prev_centroid = (self.my_centroid[0],self.my_centroid[1])
        self.prev_error = (self.error[0],self.error[1])
        while not rospy.is_shutdown():
            throttle = -0.1 #Left thumbstick Y
            steering = giro #Right thumbstick X
            #Pubblish gamepad values
            self.car.throttle.publish(throttle)
            self.car.steering.publish(steering)
            rate.sleep()

    def stop():
        throttle = 0.0
        self.car.throttle.publish(throttle)

if __name__ == '__main__':
    #Camara init
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    while(cap.isOpened()):
        ret,frame = cap.read()
        frame_b = process_image(frame)
        cropped = region_of_interest(frame_b)
        contours = getContours(cropped)
        if len(contours) > 0:
            # c = max(contours, key = cv2.contourArea)
            # print(c)
            M = cv2.moments(cropped)
            if M["m00"]!=0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                self.my_centroid = cx,cy
            cv2.circle(frame,(cx,cy),5,(0,0,255),-1)
        self.centroidToTurn()

        if np.abs(time.time()-lastPublication) > MAX_FPS:
                try:
                    my_follower.centroidToTurn()
                    # my_follower.printCentroid()
                except Exception as e:
                    print(e)
                lastPublication = time.time()

        cv2.imshow("Frame",frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    my_follower.stop()
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

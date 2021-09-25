#!/usr/bin/env python
#-*- coding:UTF-8 -*-
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy, time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
class PID:
    def __init__(self, P , I , D ):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.clear()
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/image_raw', 
                                        Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                        Twist, queue_size=1)                          
        self.twist = Twist()

    def clear(self):
        self.current_time = time.time()
        self.last_time = self.current_time
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.output = 0.0

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')  # get the image, an array
        self.geterror(image)  # get the current_error when before change the direct(move to line 38)
        self.update()
        self.control()
        #cv2.namedWindow("window", 1)
        #cv2.imshow("window", image)
        cv2.waitKey(3)
    
    def geterror(self,img):  # img is the current image of the car's camera
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # everything pixel transfer to [h,s,v],h is hue(sediao), s is saturability(baohedu), v is value(mingdu)
        lower_white = numpy.array([ 0,  0, 221])  # the edge of the lowest white
        upper_white = numpy.array([180, 30, 255])  # the edge of the highest white
        mask1 = cv2.inRange(hsv, lower_white, upper_white)  # hsv is an array like [[h,s,v],[h,s,v],[h,s,v]]
        mask2 = cv2.inRange(hsv, lower_white, upper_white)  # when the [h,s,v] is in the range of lower_white and higher_white, then this position of mask1 is 1
        # otherwise it is 0. by doing this, we get an array white all pixels with white marked as 1, all other colors marked as 0 
        h,w,d = img.shape  # add this line//////******///////
        search_top_1 = 3*h/4 + 20
        search_bot_1 = search_top_1 + 20  # 3*h/4 + 40
        search_top_2 = 3*h/4 - 80
        search_bot_2 = search_top_2 + 20  # 3*h/4 - 60
        mask1[0:search_top_1, 0:w] = 0  # cut the array to ignore useless points. right and left is important because they are related to the path's direction.
        # above we cut the bottom
        mask1[search_bot_1:h, 0:w] = 0  # this line we cut the head
        mask2[0:search_top_2, 0:w] = 0  # similar action, but change the range of the cut
        mask2[search_bot_2:h, 0:w] = 0
        # the mask1 and mask2 is like:
        M1 = cv2.moments(mask1)  # get the moment of mask1 and mask2(ju)
        M2 = cv2.moments(mask2)
        # M['mij']=Sum( array(x,y)*(x^i)*(y^j) ), visit all positions in the array
        if M1['m00'] > 0 and M2['m00'] > 0 :  # judge if the array exist 1, if not, the car is out of the road already 
            cx1 = int(M1['m10']/M1['m00'])  # the core in x
            cy1 = int(M1['m01']/M1['m00'])  # the core in y
            cx2 = int(M2['m10']/M2['m00'])
            cy2 = int(M2['m01']/M2['m00'])
            cv2.circle(img, (cx1, cy1), 10, (0,0,255), -1)  # draw the circle in the screen, the core's position. that is mask1
            cv2.circle(img, (cx2, cy2), 10, (0,0,255), -1)  # similar action, for mask2
            k = 0.10  # can change it to adjust
            self.current_error = k*cx1+(1-k)*cx2-w/2  # w/2 is the center of the pic
        else:  # if the car is out of the road
            err = self.last_error  
            self.clear()
            self.current_error = err
        
    def update(self):
        error = self.current_error  # current_error is changed in self.geterror
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time  # the change of the time
        delta_error = error - self.last_error  # the error's change during the time
        self.PTerm = self.Kp * error  # P is proportion
        self.ITerm += error * delta_time  # I is integral, use += continuely to immitate integral
        self.DTerm = 0.0  # # D is derivative
        if delta_time > 0:
            if delta_error * error > 0:  # last_error is 0.2, current error is 0.1, then delta<0,
            # means overchange
                self.DTerm = delta_error / delta_time  # during this time, the speed of the change of the error, the slope of error
                # when the change of error is too fast, we hope it slow down, when too slow, make it faster
            else:  # haven't overchange
                self.DTerm = 0.1*delta_error / delta_time
        self.last_time = self.current_time
        self.last_error = error
        self.output = self.PTerm + self.Ki*self.ITerm + self.Kd*self.DTerm  # use output as the final value to control the car
    
    def control(self):
        err = self.output
        self.twist.linear.x = speed / ( 1 + err**2 / 200)  # speed in x axis, just the speed
        self.twist.angular.z = float(err)  # speed of the change of the angular of the car, just the speed to change the direction
        self.cmd_vel_pub.publish(self.twist)  # send it

p = 0.046
i = 0.004
d = 0.0128
speed = -2.3
# (w,h) = [1280,720]

rospy.init_node('follower')  # start
follower = PID(p,i,d)
rospy.spin()
# END ALL

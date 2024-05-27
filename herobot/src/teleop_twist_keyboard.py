#!/usr/bin/env python3

from __future__ import print_function

import threading

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from herobot.msg import YoloResult
from geometry_msgs.msg import Twist

import sys, select, termios, tty

from sensor_msgs.msg import PointCloud2
import numpy as np
msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""


image_width = 640
image_height = 480

moveBindings = {      
        ## control param
        'i':(1,0,0,0), #forward
        'o':(1,0,0,-1), #turn right
        'j':(0,0,0,1), #left
        'l':(0,0,0,-1), #right
        'u':(1,0,0,1), #turn left
        ',':(-1,0,0,0), #back
        '.':(-1,0,0,1), 
        'm':(-1,0,0,-1),
        
        ### Cap
        'O':(1,-1,0,0),
        'I':(1,0,0,0),
        'J':(0,1,0,0),
        'L':(0,-1,0,0),
        'U':(1,1,0,0),
        '<':(-1,0,0,0),
        '>':(-1,-1,0,0),
        'M':(-1,1,0,0),
        
        ## Useless val
        't':(0,0,1,0),
        'b':(0,0,-1,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }



class PublishThread(threading.Thread):
    def __init__(self):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
        #self.subscriber = rospy.Subscriber('/ouster/points', PointCloud2, self.range_callback)
        self.subscriber = rospy.Subscriber('/ouster/points', PointCloud2)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False
        self.center_x = 0
        self.Bbox_sub= rospy.Subscriber("yolo_result", YoloResult, self.callback)
        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        self.speed = rospy.get_param("~speed", 0.2)
        self.turn = rospy.get_param("~turn", 0.5)
        self.repeat = rospy.get_param("~repeat_rate", 0.0)
        self.key_timeout = rospy.get_param("~key_timeout", 0.0)
        self.size_x = 0
        self.size_y = 0
        self.center_x = 0
        self.center_y = 0
        self.class_id = 0
        self.score = 0
        self.is_triggered = True
        
        ## Variables for calculate range based on lidar(CHAINFIELD.RANGE?)
        #self.range_array = [0][0]
        #self.range = 0
        
        while not rospy.is_shutdown():
            if self.is_triggered == True:
                self.control()


    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(self.publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")
    
    
    ### def for range from
    #def range_callback(self, data):
        #self.range_array = data.fields
        #self.range_array = np.asarray(self.range_array)
        #print(np.shape(data.ranges))

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        
        twist = Twist()
        twist.linear.x = self.x * self.speed
        twist.linear.y = self.y * self.speed
        twist.linear.z = self.z * self.speed
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.th * self.turn

        ## Open when you wanna check input
        #print(twist.linear.x, twist.linear.y)
        self.publisher.publish(twist)
         
        self.condition.notify()
        self.condition.release()

    #def stop(self):
    #    self.done = True
    #    self.update(0, 0, 0, 0, 0, 0)
    #    self.join()

    def run(self):
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        
    def callback(self, data):
        if len(str(data.detections)) > 90:
            #print(len(str(data.detections)))
            self.size_x = data.detections.detections[0].bbox.size_x
            self.size_y = data.detections.detections[0].bbox.size_y
            self.center_x = int(data.detections.detections[0].bbox.center.x)
            self.center_y = int(data.detections.detections[0].bbox.center.y)
            self.class_id = data.detections.detections[0].results[0].id
            self.score = data.detections.detections[0].results[0].score
        #else:
            #print("error")

    def control(self):
        #pub_thread = PublishThread(repeat)
        x = 1
        y = 0
        z = 0
        th = 1
        status = 0
        speed = 0.2
        turn = 0.5
        output = ['recog', 'back and forth', 'left and right', 0.2, 0.5, 320, 320]
        ## val for lidar data
        #range = [128][1024]
        #range = [0][0]
        try:
            self.update(x, y, z, th, speed, turn)

            #print(msg)
            #print(vels(speed,turn))
        
            while(1):
                #print(self.class_id)
                if(self.class_id != 0):
                    speed = 0
                    turn = 0
                    output[0] = "No"
                    #output = ["Human not recognized", '-', '-' , 0, 0, 0, 0]        
                         
                elif(self.size_y < 400):
                    speed = 0
                    turn = 0
                    output[0] = "No"
                    #output = ["Human not recognized", '-', '-' , 0, 0, 0, 0]                        
                
                else:
                    output[0] = "Human is recognized"
                    #ange = self.range_array[self.center_x][self.center_y]
                    #print(range)
                    if(self.center_x < (image_width/2*0.95)):
                        th = 1
                        turn = ((image_width/2) - self.center_x)/image_width*3.0
                        output[2] = "Left"      
                    elif(self.center_x > (image_width - image_width/2*0.95)):
                        th = -1
                        turn = (self.center_x - (image_width/2))/image_width*3.0
                        output[2] = "Right"
                    else:
                        turn = 0
                        output[2] = "Straight"
                    
                    if(self.size_x <image_width/2):
                        speed = ((image_width/2) - self.size_x) / (image_width/2) * 2.0
                        output[1] = "Forward"
                        x = 1
                    
                    elif(self.size_x>image_width/2* 1.05):
                        output[1] = "Stop"
                        x = 0
                        speed = 0                        
                        #output[1] = "Back"
                        #speed = 0.2
                        #x = -1
                        #th = th * -1
                    else:
                        output[1] = "Stop"
                        x = 0
                        speed = 0
                    
                    if(0 < speed < 0.2):
                        speed = 0.2
                    if(speed > 1.5):
                        speed  = 1.5
                                        
                #print(turn, speed)           
                output[3] = speed
                output[4] = turn
                output[5] = self.center_x
                output[6] = self.size_x
                print("\033[2J", end='')
                print(f"{output[0]} {output[1]} {output[2]} \nSpeed: {output[3]:.2f} Angular: {output[4]:.2f} \nCentor: {output[5]:.2f} / with: {output[6]:.2f}")
                print(f"Height: {self.size_y:.2f}")
                
                self.update(x, y, z, th, speed, turn)
              

        except Exception as e:
            print(e)

        #finally:
            #self.stop()

        #termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
# 720x540 
       
def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)


if __name__=="__main__":
    rospy.init_node('teleop_twist_keyboard', anonymous=True)
    node = PublishThread()
    node.main()


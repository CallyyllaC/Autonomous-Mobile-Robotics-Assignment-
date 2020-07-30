import rospy
import math
import get_colours
from functions import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

#class used to process the robot navigation
class navigate():
    def __init__(self):
        print "init"
        
        #Global Variables
        #ROS
        self.r = rospy.Rate(5)
        
        #Subscribers
        #sub to the lazer scan
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.update_lazer)
        #sub to the colour averages
        self.red_sub = rospy.Subscriber('/result_red', Float32, self.update_red)
        self.blue_sub = rospy.Subscriber('/result_blue', Float32, self.update_blue)
        self.green_sub = rospy.Subscriber('/result_green', Float32, self.update_green)
        #sub to the direction to targets
        self.blue_sub = rospy.Subscriber('/to_blue', Twist, self.update_dir_blue)
        self.green_sub = rospy.Subscriber('/to_green', Twist, self.update_dir_green)

        #Publishers
        #publish movement to the robot
        self.publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        #Start up the robots eyes
        self.colour = get_colours.eyes()
        #create variables to hold the colour subscribers
        self.green = 0
        self.red = 0
        self.blue = 0
        self.dir_green = Twist()
        self.dir_blue = Twist()
        #create variable to hold the lazer data
        self.lazerarr = []
        #test to see if this is the first run
        self.firstcall = False
        self.firstrun = False
        #found a colour so ignore most logic for now
        self.foundsomething = False
        #time since lass observation check
        self.lastcheck = 0
        #cool down from finding blue
        self.lastblue = 0
        self.lastturn = "none"
        #get the distance between each point in the las=zer array in radians
        self.rad_per_point = 0
                

    #Callback functions

    #update the local variable for lazer
        #between 0 and 640 pointers
        #lazer look from -30 to 30 degrees
        #lazer will return values between 0.45m and 10m or it will return nan
    def update_lazer(self, laser_msg):  

        #if first call get the lazer data
        if(self.firstcall == False):
            self.firstcall = True
            self.rad_per_point = laser_msg.angle_increment
        #update the local lazer array
        self.lazerarr = laser_msg.ranges

    #get the twist object to the goal
    def update_dir_green(self, msg):
        self.dir_green = msg

    #get the twist object to the clue
    def update_dir_blue(self, msg):
        self.dir_blue = msg

    #get the mean green value
    def update_green(self, msg):
        self.green = msg.data

    #get the mean blue value
    def update_blue(self, msg):
        self.blue = msg.data

    #get the mean red value
    def update_red(self, msg):
        self.red = msg.data



    #Main move logic
    def move(self):

        #if lazer array is empty return
        if not self.lazerarr:
            return
        
        if self.firstrun == False:
            self.firstrun = True
            self.check_for_something(360) #spin around and look for blue
        
        ##ADD CHECK IF STUCK
        
        #Twist container object
        t = Twist()

        #hold that left and right array slices for average value more than 1m
        left = check_slice_more(self.lazerarr[0:200], 1)
        right = check_slice_more(self.lazerarr[440:640], 1)

        #If there is a wall infront
        if check_slice_less(self.lazerarr[240:400], 0.8):
            #Check if the robot is in a wall (<0.45m away from a wall)
            if checknan(self.lazerarr[315:325]):
                print "Im stuck!"
                t.linear.x = -0.1 #reverse

            #if at a T junction look left and right (cooldown to stop repeating)
            elif left and right and self.lastcheck == 0:
                print "no wall left or right"
                if self.check_for_something(-90):
                    print "found something left"
                    t.angular.z = math.radians(-90)

                elif self.check_for_something(90):
                    print "found something right"
                    t.angular.z = math.radians(90)


            #Check if there is a wall to the left turn left and remember
            elif left:
                print "no wall left"
                t.angular.z = math.radians(-45)
                self.lastturn = "left"
            
            #Check if there is a wall to the right turn right and remember
            elif right:
                print "no wall right"
                t.angular.z = math.radians(45)
                self.lastturn = "right"
            
            #Check if there is a wall infront but closer if so, reverse
            elif check_slice_less(self.lazerarr[240:400], 0.45):#im about to walk into a wall
                print "Im about to walk into a wall"
                t.linear.x = -0.1
            
            #if the average left side is bigger than the average right side (both with nan values removed)
            elif average(repnan(self.lazerarr[0:200])) > average(repnan(self.lazerarr[440:640])):
                    print "comparing left and right"
                    t.angular.z = math.radians(-90) #turn left
                    self.lastturn = "left"

            #if the average left side is smaller than the average right side (both with nan values removed)
            elif average(repnan(self.lazerarr[0:200])) < average(repnan(self.lazerarr[440:640])):
                    print "comparing left and right"
                    t.angular.z = math.radians(90) #turn right
                    self.lastturn = "right"

            #Help me I'm really confused (the bot is not in a situation it recognises)
            else:
                print"ye gods im stuck:  left: ", average(repnan(self.lazerarr[0:200])), " compared to right: ",average(repnan(self.lazerarr[440:640])) #print debug data
                if self.lastturn == "left":         #if last turn was left, keep turning left
                    self.lastturn = "left"
                    t.angular.z = math.radians(-90)

                elif self.lastturn == "right":      #if last turn was right keep turning right
                    self.lastturn = "right"
                    t.angular.z = math.radians(90)
                
                else:
                    t.angular.z = math.radians(-90) #something happened to my last turn so just go left

        #If there isnt a wall infront (atleast that it detects)
        else:

            #Check if the robot is in a wall (<0.45m away from a wall)
            if checknan(self.lazerarr[315:325]):
                print "Im stuck!"
                t.linear.x = -0.1 #reverse

            elif self.green > 1: #if I see the exit
                t = self.dir_green #get the angle needed
                t.linear.x = 0.2
                print "I see green" ,self.green

            elif self.red > 2: #if I see a trap
                print "It's a trap!" ,self.red
                t.angular.z = math.radians(-180) #turn around

            elif self.blue > 1 and self.blue < 40: #if I see the guide
                t = self.dir_blue #get the angle needed
                t.linear.x = 0.2
                print "I see blue" ,self.blue

            elif self.blue > 40: #okay close enough to that guide there
                #start ignoring the blue for a while
                self.lastblue = 200
                #if there is a gap left go left
                if left:
                    self.lastturn = "left"
                    t.angular.z = math.radians(-90)
                #if there is a gap right go right
                elif right:
                    self.lastturn = "right"
                    t.angular.z = math.radians(90)

            elif check_slice_less(self.lazerarr[0:20], 0.45): #getting a bit close to the left wall, straighten up
                t.angular.z = math.radians(25)

            elif check_slice_less(self.lazerarr[620:640], 0.45): #getting a bit close to the right wall, straighten up
                t.angular.z = math.radians(-25)

            #nothing to add, keep going forward
            else:
                t.linear.x = 0.2
                t.angular.z = 0

        if self.lastcheck <= 0:
            self.lastcheck = 0
        if self.lastblue <= 0:
            self.lastblue = 0
        self.lastcheck = self.lastcheck - 1
        self.lastblue = self.lastblue - 1
        self.publisher.publish(t)
        self.r.sleep()




    #if I come to a T junction look left and right for a clue
    def check_for_something(self, angle):
        #set last checked time
        self.lastcheck = 200 
        print "checking ", angle, "..." #debug
        nothing = False #return bool
        add = 0 #hold addition
        sub = 0 #hold subtration
        mult = 1    #multiplier
        if angle < 0: #if angle is less than 0 set it negative
            mult = -1

        #while nothing is found
        while(not nothing):
            #new twist
            t = Twist()

            #if checked return nothing found
            if sub >=  abs(angle):
                return False

            #if ive turned one way, turn back
            if add >= abs(angle):
                t.angular.z = math.radians(15*mult)
                sub = sub+3 #add 3 rather than one as its aproximate to the friction and acceleration of the robot (not perfect but good enough)

            #im doing the intial turn
            else:
                t.angular.z = math.radians(-15*mult)

            
            if self.green > 1: #if I see the exit more towards it and return
                print "I see green" ,self.green
                t = self.dir_green
                t.linear.x = 0.2
                self.publisher.publish(t)
                return True

            elif self.blue > 1 and self.blue < 5: #if I see the guide more towards it and return
                print "I see blue" ,self.blue
                t = self.dir_blue
                t.linear.x = 0.2
                self.publisher.publish(t)
                return True

            #publish and loop
            self.publisher.publish(t)
            self.r.sleep()
            add = add + 3

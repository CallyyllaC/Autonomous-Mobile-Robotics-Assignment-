import rospy
import navigate

#main class
class MazeNavigator5000:

    def __init__(self):
        #ROS
        rospy.init_node('MazeNavigator5000')
        self.n = navigate.navigate()
        
    #Main loop
    def run(self):
        while not rospy.is_shutdown():
            self.n.move()
        rospy.spin()



if __name__ == '__main__':
    robot = MazeNavigator5000()
    robot.run()
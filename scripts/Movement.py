import rospy, random, actionlib, time
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from numpy import *
from topological_navigation_msgs.msg import GotoNodeAction, GotoNodeGoal

class Robotic_Movement:
# Laser range (-2.18 - 2.36) want to keep the orientation front, therefore front_scan might be only needed.
# -124.9 + 135.22 range with an increment of 0.351 degrees. Can look at data ranges grabbed from laserscan front
# 720 data ranges found, therefore suggesting 720 increments instead... Posing the first and last range into visualisation
# will help for finding left position and right position to an accept degree.
    avoid_dist = 2.3
    LR = random.randint(1,2)
    def __init__(self, mselect):
        self.publisher = rospy.Publisher('thorvald_001/teleop_joy/cmd_vel', Twist ,queue_size=1)
        self.message = rospy.Publisher('toponav_movement_complete', String, queue_size=1)
        self.node_update = rospy.Publisher('node_pos', String, queue_size=1)
        self.rate = rospy.Rate(1)
        if 'random' in mselect:
            print('Random algo selected: Initialised... ')
            rospy.Subscriber('thorvald_001/front_scan', LaserScan, self.R_algo)
            rospy.spin()
        elif 'hugging' in mselect:
            print('Hugging algo selected: Initialised... ')
            rospy.Subscriber('thorvald_001/back_scan', LaserScan, self.back_scan)
            rospy.Subscriber('thorvald_001/front_scan', LaserScan, self.H_algo)
            rospy.spin()
        elif 'stopping' in mselect:
            print('Stopping: Initialised... ')
            rospy.Subscriber('thorvald_001/back_scan', LaserScan, self.back_scan)
            rospy.Subscriber('thorvald_001/front_scan', LaserScan, self.stop)
            rospy.spin()
        elif 'topo' in mselect:
            print('Topological Navigation: Initialised...')
            ## from set topo_nav goal...
            self.topo()

    def R_algo(self, data):
        '''
        This was based on the mover.py script but a little more indepth, however same principal.
        Useful for getting the correct localisation positions and gmapping for mapping areas.
        However below the hugging algorithm mentioned in FoR might be of more use and is more intricate in design.
        '''
        range, twist = array(data.ranges), Twist()
        front_area_coverages , left_area_coverages, right_area_coverages = range[251:450], range[451:719], range[0:250]
        if any((front_area_coverages < self.avoid_dist)) != True or any((left_area_coverages < self.avoid_dist)) != True or any((right_area_coverages < self.avoid_dist)) != True:
            twist.linear.x = 1.0
        elif any((left_area_coverages < (self.avoid_dist))) or any((front_area_coverages < (self.avoid_dist))):
            twist.linear.x = 0.0
            twist.angular.z = 0.4
        elif any((right_area_coverages < (self.avoid_dist))) or any((front_area_coverages < (self.avoid_dist))):
            twist.linear.x = 0.0
            twist.angular.z = 0.4
        self.publisher.publish(twist)

    def H_algo(self, data):
        '''
        https://github.com/nimbekarnd/Wall-follower-in-ROS-using-Python/blob/main/src/motion_plan/nodes/follow_wall.py
        above was helpful for more understanding however I made some personal changes.
        Wall hugging requires a wall to the left or right. If no wall we need to simulate a wall being left or right
        till one is found in either direction, this selection doesn't matter and thus I'll add some randomness
        NOTE: not gonna lie - this is a horrid method and gave me a headache :) doesn't work good... needs more time, ill return to it ...
        '''
        range, twist, critical_dist = array(data.ranges), Twist(), 1.00
        front_area_coverages , left_area_coverages, right_area_coverages = range[251:450], range[451:], range[:250]
        back_area_coverages, bleft_area_coverages, bright_area_coverages = brange[340:360], brange[361:], brange[:339]
        ## west wall movements
        if any((front_area_coverages < critical_dist)) != True and any((left_area_coverages < critical_dist)) != True:
            if self.LR == int(1):
                twist.linear.x = 0.01
                twist.linear.y = 1.4
            if any((bright_area_coverages < critical_dist-0.3)) or any((left_area_coverages < (critical_dist-0.3))):
                twist.linear.y = -0.1
                twist.linear.x = 1.0
            if any((left_area_coverages >= critical_dist)) and any((left_area_coverages <= self.avoid_dist)) and any((bright_area_coverages > 0.75)) and any((bright_area_coverages < self.avoid_dist)):
                twist.linear.x = 1.0
        ## north wall movements
        elif any((front_area_coverages <= (self.avoid_dist))) and any((bleft_area_coverages > (self.avoid_dist+3))):
            if self.LR == int(1):
                twist.linear.x = 0.1
                twist.linear.y = -1.4
            if any((front_area_coverages < critical_dist)) or any((left_area_coverages < critical_dist)) or any((right_area_coverages < critical_dist)) :
                twist.linear.x = -0.025
                twist.linear.y = -1.0
        ## east wall movements  ---- SOEMTHING WRONG FIX IT
        elif any((right_area_coverages < self.avoid_dist)) and any((bleft_area_coverages > (self.avoid_dist+0.5))) and any((bright_area_coverages > self.avoid_dist)):
            if self.LR == int(1):
                twist.linear.x = -1.4
                twist.lineat.y = 0.02
            if any((right_area_coverages < self.avoid_dist)) or any((front_area_coverages < self.avoid_dist)) or any((bleft_area_coverages < (self.avoid_dist + 0.35))):
                twist.linear.x = -1.0
                twist.linear.y = -0.05

            if any((right_area_coverages < critical_dist)) or any((bleft_area_coverages < (critical_dist + 0.35))):
                twist.linear.x = -1.0
                twist.linear.y = -0.1
        ## south wall movements
        elif any((back_area_coverages < critical_dist)) or any((bleft_area_coverages < critical_dist)):
                twist.linear.x = 0.0
                
        self.publisher.publish(twist)
    
    def stop(self, data):
        range, twist = array(data.ranges), Twist()
        front, back = range[300:500], brange[300:500]
        if any((front < 0.5)) or any((back < 0.5)):
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
    
    def topo(self):
        ## Used for vineyard map vineyard_small_s4_coarse
        ## Used from the LCAS wiki... with a small modification
        client = actionlib.SimpleActionClient('/thorvald_001/topological_navigation', GotoNodeAction)
        client.wait_for_server()
        all_iter_time = time.time()
        loop = 1
        for j in range(loop):
            one_iter_time = time.time()
            # 4 loops to get an average...
            for i in range(10, 34):
                goal = GotoNodeGoal()
                goal.target = 'WayPoint'+str(i)
                rospy.loginfo("going to %s", goal.target)
                client.send_goal(goal)
                status = client.wait_for_result()
                result = client.get_result()
                rospy.loginfo("status is %s", status)
                rospy.loginfo("result is %s", result)
                self.node_update.publish(f'WayPoint'+str(i))
            elapsed_time = time.time() - one_iter_time
            print('One loop time: ', elapsed_time, ' in seconds.')
        end_time = time.time() - all_iter_time
        print(f'Time for {loop} loops: ', end_time, ' in seconds')
        self.message.publish('Topo_Nav complete. ')

    def back_scan(self, data):
        global brange
        brange = array(data.ranges)

def main(inp):
    if inp.lower() == 'random':
        Robotic_Movement('Movement_random')
    elif inp.lower() == 'hugging':
        Robotic_Movement('Movement_hugging')
    elif inp.lower() == 'stopping':
        Robotic_Movement('Movement_stopping')
    elif inp.lower() == 'topo':
        Robotic_Movement('Movement_topo')

if __name__ == '__main__':
    rospy.init_node('Robot_movement')
    main(input('Random or Hugging or Stopping or Topo?: '))
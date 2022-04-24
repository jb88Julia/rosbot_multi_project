#! /usr/bin/env python
from difflib import get_close_matches
import queue
import rospy
# for the path planning
from enum import Enum
import actionlib
from queue import Queue
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult, MoveBaseActionFeedback


class Robot_State(Enum):
    idle = 1
    lost = 2
    searching = 3
    rescued = 4
    lost_4_ever = 5


# Node that will control the whole flow of the mission:
# -request move_base actions from each robot
# -get statuses of each robot
class RobotCore:

    robot_name = None
    node_name = None
    mb_client = None
    status = None
    current_path = None

    def __init__(self, robot_name:str) -> None:
        
        self.robot_name = robot_name
        self.node_name =  robot_name+'_core'
        self.status =Robot_State.idle
        self.current_path = Queue(maxsize=20)

        # create the connections to the Move Base action server
        self.connect_move_base()


    def connect_move_base(self):
        # create the connections to the Move Base action server
        self.mb_client = actionlib.SimpleActionClient(self.robot_name + '/move_base', MoveBaseAction)
        # waits until the action server is up and running
        self.mb_client.wait_for_server()

    def newGoal(self, goal:dict):
        new_goal = MoveBaseGoal() 
        #new_goal.target_pose.header.stamp = rospy.Time.now()
        new_goal.target_pose.header.frame_id = 'map'
        new_goal.target_pose.pose.position.x = goal.get("x")
        new_goal.target_pose.pose.position.y = goal.get("y")
        new_goal.target_pose.pose.position.z = goal.get("z")
        new_goal.target_pose.pose.orientation.x = goal.get("roll")
        new_goal.target_pose.pose.orientation.y = goal.get("pitch")
        new_goal.target_pose.pose.orientation.z = goal.get("yaw")
        new_goal.target_pose.pose.orientation.w = goal.get("w")
        return new_goal

    def add_many_goals(self, goals_list:list):
        for goal_dict in goals_list:
            new_goal = self.newGoal(goal_dict)
            self.current_path.put_nowait(new_goal)
            #rospy.logwarn("puttin x: %s and y:%s in queue for %s", str(goal_dict.get("x")),str(goal_dict.get("y")), self.robot_name)

    def add_one_goal(self, goal:dict):
        new_goal = self.newGoal(goal)
        self.current_path.put_nowait(new_goal)

    def get_next_goal(self):
        goto = self.current_path.get_nowait()
        goto.target_pose.header.stamp = rospy.Time.now()
        return goto

    #def move_to_next(self):
    #    goto = self.current_path.get_nowait()
    #    goto.target_pose.header.stamp = rospy.Time.now()


class Mission():
    def __init__(self) -> None:
        rospy.init_node("mission_control")
        self.r1 = RobotCore("robot1")
        self.r2 = RobotCore("robot2")
        self.r3 = RobotCore("robot3")
        self.rescue_team = [self.r1,self.r3]
        rospy.logwarn("Starting mission... Robot 2 getting lost")
        

    def start(self):
        #Start mission by getting r2 lost:
        self.get_lost(self.r2)

        while self.r2.status != Robot_State.lost:
            rospy.loginfo("Robot 2 getting lost")
            rospy.sleep(2)

        self.start_rescue()

        while self.r2.status != Robot_State.lost_4_ever:
            rospy.loginfo("Rescue mission in motion")
            rospy.sleep(2)    

        self.terminate_mission()


    def get_lost(self, robot:RobotCore):
        # Start mission
        lost_params = rospy.get_param("lost_coords")
        robot.add_one_goal(lost_params)

        #send current goal to r2_move_base:
        try:
            goto = robot.get_next_goal()
        except queue.Empty:
            rospy.loginfo("emty queue on robot 2")

        robot.mb_client.send_goal(goto, done_cb=self.robot_lost)

        return

    def robot_lost(self, goalStatus, result):
        self.r2.status = Robot_State.lost
        rospy.logwarn("Oh no!! Robot 2 is now lost")
        rospy.logwarn("Sending Robot 1 and 3 for search mission")
    
    
    def start_rescue(self):
            
        r1_rescue_path = [rospy.get_param("r1_goal1"), rospy.get_param("r1_goal2")]
        r3_rescue_path = [rospy.get_param("r3_goal1"), rospy.get_param("r3_goal2")]

        self.r1.add_many_goals(r1_rescue_path)
        self.r3.add_many_goals(r3_rescue_path)

        self.r1.status = Robot_State.searching
        self.r3.status = Robot_State.searching

        try:
            #r1g=self.r1.get_next_goal()
            self.r1.mb_client.send_goal(self.r1.get_next_goal(), done_cb= self.r1_goal_reached)
            #rospy.logwarn("Sending R1 to %s, %s", str(r1g.target_pose.pose.position.x), str(r1g.target_pose.pose.position.y))
            #r3g=self.r3.get_next_goal()
            self.r3.mb_client.send_goal(self.r3.get_next_goal(), done_cb= self.r3_goal_reached)
            #rospy.logwarn("Sending R3 to %s, %s", str(r3g.target_pose.pose.position.x), str(r3g.target_pose.pose.position.y))
        except queue.Empty:
            rospy.logfatal("No plan for rescue specified")

    def r1_goal_reached(self, goalStatus, result):
        rospy.logwarn("R1 reached the one of the goals")

        try:
            #r1g=self.r1.get_next_goal()
            self.r1.mb_client.send_goal(self.r1.get_next_goal(), done_cb= self.r1_goal_reached)
            #rospy.logwarn("Sending R1 to %s, %s", str(r1g.target_pose.pose.position.x), str(r1g.target_pose.pose.position.y))
            
        except queue.Empty:
            self.r1.status = Robot_State.idle
            rospy.logwarn("No further goals for R1")
            rospy.logwarn("R1 going idle for now")

            if self.r3.status == Robot_State.idle and self.r2.status == Robot_State.lost:
                self.r2.status = Robot_State.lost_4_ever

            else:
                rospy.logwarn("R3 is still searching")


    def r3_goal_reached(self, goalStatus, result):
        rospy.logwarn("R3 reached reached one of the goals")

        try:
            #r3g=self.r3.get_next_goal()
            self.r3.mb_client.send_goal(self.r3.get_next_goal(), done_cb= self.r3_goal_reached)
            #rospy.logwarn("Sending R3 to %s, %s", str(r3g.target_pose.pose.position.x), str(r3g.target_pose.pose.position.y))
        except queue.Empty:
            self.r3.status = Robot_State.idle
            rospy.logwarn("No further goals for R3")
            rospy.logwarn("R3 going idle for now")

            if self.r1.status == Robot_State.idle and self.r2.status == Robot_State.lost:
                self.r2.status = Robot_State.lost_4_ever

            else:
                rospy.logwarn("R1 is still searching")


    def terminate_mission(self):

        rospy.logwarn("~~~~~~~ Mission finished~~~~~~~~")
        rospy.logwarn("Status:  %s", str((self.r2.status).name))
        rospy.spin()
                


if __name__=="__main__":
    #if len(sys.argv) < 2:
    #    print("usage: robot_core.py robot_name")
    #else:
    myMission = Mission()
    myMission.start()


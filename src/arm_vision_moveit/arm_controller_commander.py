import rospy
import moveit_commander
import moveit_msgs.msg
import sys

from rpi_arm_composites_manufacturing_abb_egm_controller.srv import \
    SetControllerMode, SetControllerModeRequest, SetControllerModeResponse
    
from rpi_arm_composites_manufacturing_abb_egm_controller.srv import \
    RapidStart, RapidStartRequest, RapidStartResponse, \
    RapidStop, RapidStopRequest, RapidStopResponse, \
    RapidGetStatus, RapidGetStatusRequest, RapidGetStatusResponse, \
    RapidGetDigitalIO, RapidGetDigitalIORequest, RapidGetDigitalIOResponse, \
    RapidSetDigitalIO, RapidSetDigitalIORequest, RapidSetDigitalIOResponse, \
    RapidReadEventLog, RapidReadEventLogRequest, RapidReadEventLogResponse


class ARMControllerCommander:
    def __init__(self):
        
        self.set_controller_mode=rospy.ServiceProxy('set_controller_mode', SetControllerMode)
        self.set_digital_io=rospy.ServiceProxy('rapid/set_digital_io', RapidSetDigitalIO)
    def move_it_init(self,moveit_group_name,goal_position_tolerance,replanning,planner_id,num_planning_attempts):
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('collision_checker','move_group_python_interface_tutorial', anonymous=True)
        
        ## MoveIt! Initialization
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander(moveit_group_name)
        group.set_goal_position_tolerance(goal_position_tolerance)
        group.allow_replanning(replanning)
		#specifies which path planner to use for the Moveit
        group.set_planner_id(planner_id) #RRTConnectkConfigDefault/SBLkConfigDefault/KPIECEkConfigDefault/BKPIECEkConfigDefault/LBKPIECEkConfigDefault/
        group.set_num_planning_attempts(num_planning_attempts)
        self.display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)
        return group
    
    def set_controller(self,mode,speed_scalar,ft_threshold):
        req=SetControllerModeRequest()
        req.mode.mode=mode
        req.speed_scalar=speed_scalar
        req.force_torque_stop_threshold=ft_threshold
        res=self.set_controller_mode(req)
        if (not res.success): raise Exception("Could not set controller mode")

    def set_vacuum(self,status):
        #send 0 or 1 to change vacuum
        req=RapidSetDigitalIORequest()
        req.signal="Vacuum_enable"
        req.lvalue=status
        self.set_digital_io(req)




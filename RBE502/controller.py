import rclpy
from std_msgs.msg import String
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode 

class Controller(Node):

    def __init__(self):
        #Initialize the controller and connect to all of the relevant information provided by XRCE
        super().__init__('controller')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )  #QOS profiles are stupid and i hate them and we never use anything but the default at work, but the XRCE app is fancy and decided to use this one. Nothing we need to be concerned with
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status',self.vehicle_status_callback, qos_profile)

        
        self.publish_arm_command = self.create_publisher(VehicleCommand,'/fmu/in/vehicle_command', qos_profile)
        #Publisher to tell the AP what mode to be in and what type of setpoints to expect (position, velocity, etc)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        #the actual position/velocity/etc setpoint publisher
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        #How often we're gonna run our control loop. It needs to run a MINIMUM of 2Hz to be considered 'alive' by the AP
        #1 Hz = 1/seconds 
        timer_period = 0.02  # seconds. Starting at about 50Hz and we can decrease the refresh rate as needed based on how computationally expensive our app is
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        self.nav_state=0
        self.arming_state=0

        self.parse_params()
        self.arm()

    def parse_params(self):        
        #TODO update these as we go. declare a parameter for each GLOBAL variable 
        self.declare_parameter('radius', 10.0)  #Parameter name and then default value
        self.declare_parameter('omega', 5.0)    #Technically, in a real-world scenario these would be specified  in a .yaml 
        self.declare_parameter('altitude', 5.0) #config file at runtime, but we dont need to be that fancy for this project.

        # Note: no parameter callbacks are used to prevent sudden inflight changes of radii and omega 
        # which would result in large discontinuities in setpoints
        self.theta = 0.0
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value


    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("NAV_STATUS: ", msg.nav_state)
        print("  - offboard status: ", VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        print("  - arm status: ", msg.arming_state)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state


    def cmdloop_callback(self):
        # Publish offboard control modes. Set whatever we're controlling to TRUE, everything unnecessary to FALSE
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=True        
        offboard_msg.velocity=False
        offboard_msg.acceleration=False
        self.publisher_offboard_mode.publish(offboard_msg)
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):

            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = self.radius * np.cos(self.theta)
            trajectory_msg.position[1] = self.radius * np.sin(self.theta)
            trajectory_msg.position[2] = -self.altitude
            self.publisher_trajectory.publish(trajectory_msg)

            self.theta = self.theta + self.omega * self.dt

        else:
            try:
                self.arm()
                msg = VehicleCommand()
                msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
                msg.param1 = 1.
                msg.param2 = 6.
                self.publish_arm_command.publish(msg)

            except:
                print("couldn't turn on offboard mode yet")

    def arm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.
        self.publish_arm_command.publish(msg)

    def disarm(self):
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.
        self.publish_arm_command.publish(msg)

    def group_controller(self, control_inputs): #errors, current state, etc. IN 
        #TODO: OUR CODE HERE 
        #we'll return our controller OUTPUTS and use the timer callback function
        #to send the responses back to the AP (autopilot) at the specified frequency
        #basically the timer will run every 0.02 seconds, call THIS function to get 
        #updated control output information and then send it out
        return [0, 0, 10]

    

def main(args=None):
    rclpy.init(args=args)

    controller = Controller()

    rclpy.spin(controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller.destroy_node()
    controller.disarm()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
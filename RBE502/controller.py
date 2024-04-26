import rclpy
from std_msgs.msg import String
import numpy as np
import math
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleControlMode 
from px4_msgs.msg import VehicleLocalPosition

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

        #Publisher for the purpose of arming and disarming the quad so we can control it
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
        
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)

        self.nav_state=0
        self.arming_state=0
        
        self.vehicle_attitude = np.empty(4)
        self.vehicle_local_position = np.empty(3)
        self.vehicle_local_velocity = np.empty(3)

        self.cube = np.array([[0,0,-5],[5,5,-5], [10,5,-5], [10,10,-5], [5,10,-5], [5,5,-5], [5,5,-10], [10,5,-5], [10,10,-10], [5,10,-10], [5,5,-10]])
        self.setpoint_num = 0
        self.des_position = self.cube[self.setpoint_num]

        self.position_error=self.des_position
        
        self.parse_params()
        self.heartbeats=0
        #self.arm()

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
        print("    Position Error: ", self.position_error)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def vehicle_local_position_callback(self, msg):
        # TODO: handle NED->ENU transformation 
        self.vehicle_local_position[0] = msg.x
        self.vehicle_local_position[1] = msg.y
        self.vehicle_local_position[2] = msg.z
        self.vehicle_local_velocity[0] = msg.vx
        self.vehicle_local_velocity[1] = msg.vy
        self.vehicle_local_velocity[2] = msg.vz
        print("     position: ", self.vehicle_local_position)
        print("     velocity: ", self.vehicle_local_velocity)



    def cmdloop_callback(self):
        # Publish offboard control modes. Set whatever we're controlling to TRUE, everything unnecessary to FALSE
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        offboard_msg.position=False        
        offboard_msg.velocity=True
        offboard_msg.acceleration=False
        offboard_msg.attitude=False
        offboard_msg.body_rate=False
        offboard_msg.thrust_and_torque=False
        offboard_msg.direct_actuator=False
        self.publisher_offboard_mode.publish(offboard_msg)
    
        #Determine if we've hit the last setpoint. If yes, then set the setpoint to the next one in the series. If no, set all velocities to 0 to hover at the last setpoint.
        ##If we're pretty darn close ( less than 0.01 m? we can play with that a little) and the controller is responding to that (velocity setpoints are getting smaller as we converge - not overshooting like crazy ans settling on the point) then we can move on
        velocity_vector_magnitude = np.linalg.norm(self.vehicle_local_velocity)
        
        if (np.average(np.array(abs(self.vehicle_local_position - self.des_position))) <=0.05 and velocity_vector_magnitude <= 0.05): 
            #if we're not on the last setpoint already
            
            if self.setpoint_num <= self.cube.size()[0] -1:
                self.setpoint_num = self.setpoint_num + 1
                self.des_position = self.cube[self.setpoint_num]
        else:
            self.des_velocity= [0,0,0]    
        
        if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            updated_velocities = self.group_controller(self.vehicle_local_position, self.vehicle_local_velocity, self.des_position, self.des_velocity)
            print("     Controller Out: ",updated_velocities)
            
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.timestamp = int(Clock().now().nanoseconds / 1000)
                        
            trajectory_msg.velocity = np.array(updated_velocities,dtype=np.float32)
            print(trajectory_msg)
            #trajectory_msg.position = np.array(self.des_position,dtype=np.float32)

            self.publisher_trajectory.publish(trajectory_msg)
            self.des_velocity = updated_velocities
            

        else:
            if self.heartbeats <= 10:
                self.publish_offboard_control_heartbeat_signal()
                self.heartbeats += 1
                print("couldn't turn on offboard mode yet")
            else:
                self.arm()
                msg = VehicleCommand()
                msg.timestamp = int(Clock().now().nanoseconds / 1000)
                msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
                msg.param1 = 1.
                msg.param2 = 6.
                self.publish_arm_command.publish(msg)

            #except:
            

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_offboard_mode.publish(msg)
    
    def arm(self):
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.
        self.publish_arm_command.publish(msg)

    def disarm(self):
        msg = VehicleCommand()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.
        self.publish_arm_command.publish(msg)

    def group_controller(self, position, velocity, des_pos, des_vel): #errors, current state, etc. IN 
        #TODO: OUR CODE HERE 
        #we'll return our controller OUTPUTS and use the timer callback function
        #to send the responses back to the AP (autopilot) at the specified frequency
        #basically the timer will run every 0.02 seconds, call THIS function to get 
        #updated control output information and then send it out
        kp = np.diag([5.5, 5.5, 35])
        kd = np.diag([1.2, 1.2, 2])
              
        e = des_pos - position
        de = des_vel - velocity
        
        output = kp @ e + kd @ de 

        self.position_error = e

        return output  #The output is goes directly into the velocity inputs of the autopilot

    

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

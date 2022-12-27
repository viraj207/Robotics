#Viraj Patel
#Student ID 201351707

from controller import Supervisor, Node

import math
import pose
from enum import Enum

class MoveState(Enum):
    STOP = 0
    FORWARD = 1
    ARC = 2
    WANDER = 3
    FOLLOW_WALL = 4

class PioneerNavX:
    """ A custom class to initialise and manage simple navigation on a Pioneer Adept robot """

    WHEEL_RADIUS = 0.0957 # in meters - found using CONFIGURE
    AXEL_LENGTH = 0.323   # in meters- found using CONFIGURE
    #def __init__(self, robot, init_pose, ps):        
    #     self.prox_sensors = ps       # reference to proximity sensors
          
    def __init__(self, robot,init_pose,ps):
        
        self.prox_sensors = ps
        
        self.robot = robot                        # reference to the robot
        self.robot_node = self.robot.getSelf()    # reference to the robot node
        self.robot_pose = self.get_real_pose()
        self.state = MoveState.STOP
        
        #initialising parameters for PID
        self.prev_error = 0
        self.total_error = 0
    
        # enable motors
        self.left_motor = self.robot.getDevice('left wheel')
        self.right_motor = self.robot.getDevice('right wheel')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        timestep = int(robot.getBasicTimeStep())
        
        self.max_vel = self.left_motor.getMaxVelocity() - 0.1 
        
        self.robot_pose = pose.Pose(init_pose.x, init_pose.y, init_pose.theta)
                
        # Initialise motor velocity
        self.left_motor.setVelocity(1.0)
        self.right_motor.setVelocity(0.0)       
    
    def get_real_pose(self):
        if self.robot_node is None:
            return pose.Pose(0, 0, 0)
            
        real_pos = self.robot_node.getPosition()
        rot = self.robot_node.getOrientation()
        theta = math.atan2(-rot[0], rot[3])
        halfpi = math.pi / 2
        theta2 = theta + halfpi
        if (theta > halfpi):
            theta2 = -(3 * halfpi) + theta
        
        return pose.Pose(real_pos[0], real_pos[1], theta2)


    # The following code is based on the avoid obstacle code supplied by the Webots
    # platform for the ePuck and allows the robot to wander randomly around the arena
    def wander(self, prox_sensors, robot_linearvelocity):
        wheel_av = (robot_linearvelocity/self.WHEEL_RADIUS)
        left_vel = wheel_av;
        right_vel = wheel_av;
        
        # detect obstacles
        right_obstacle = \
            prox_sensors.get_value(4) < 0.30 or \
            prox_sensors.get_value(5) < 0.25 or \
            prox_sensors.get_value(6) < 0.20 or \
            prox_sensors.get_value(7) < 0.15
        left_obstacle = \
            prox_sensors.get_value(0) < 0.15 or \
            prox_sensors.get_value(1) < 0.20 or \
            prox_sensors.get_value(2) < 0.25 or \
            prox_sensors.get_value(3) < 0.30

        if (left_obstacle):
            right_vel = -left_vel
        elif (right_obstacle):
            left_vel = -right_vel
                
        self.left_motor.setVelocity(left_vel)
        self.right_motor.setVelocity(right_vel)
        self.state = MoveState.WANDER
    
    
    def forward(self, target_dist, robot_linearvelocity):
        wheel_av = (robot_linearvelocity/self.WHEEL_RADIUS)
        target_time = target_dist/robot_linearvelocity
        
        self.left_motor.setVelocity(wheel_av)
        self.right_motor.setVelocity(wheel_av)
        self.state = MoveState.FORWARD
        
        # return target_time as millisecs          
        return 1000.0*target_time

    def arc(self, icr_angle, icr_r, icr_omega):
        target_time = icr_angle / icr_omega

        # Calculate each wheel velocity around ICR
        vl = icr_omega * (icr_r - (self.AXEL_LENGTH / 2))
        vr = icr_omega * (icr_r + (self.AXEL_LENGTH / 2))
        
        leftwheel_av = (vl/self.WHEEL_RADIUS)
        rightwheel_av = (vr/self.WHEEL_RADIUS)
        
        self.left_motor.setVelocity(leftwheel_av)
        self.right_motor.setVelocity(rightwheel_av)
        self.state = MoveState.ARC

        # return target_time as millisecs          
        return 1000.0*target_time            

    def stop(self):   
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.state = MoveState.STOP
        
    def set_velocity(self, base, control=0):

        base_av = (base/self.WHEEL_RADIUS)
    
        if (control != 0):
            control_av = (control/self.WHEEL_RADIUS)
            correction = 1
            lv = base_av - control_av
            rv = base_av + control_av
        
            if (lv > self.max_vel):
                correction = self.max_vel / lv
                lv = lv * correction
                rv = rv * correction
            
            if (rv > self.max_vel):
                correction = self.max_vel / rv
                lv = lv * correction
                rv = rv * correction
                                        
        else:
            lv = rv = base_av
                
        self.left_motor.setVelocity(lv)
        self.right_motor.setVelocity(rv)
        
    def pid(self, error):
        kp = 0.6 # proportional weight
        kd = 3.0# differential weight
        ki = 0.0 # integral weight 
        
        prop = error
        diff = error - self.prev_error
        self.total_error += error
        control = (kp * prop) + (ki * self.total_error) + (kd * diff)
        self.prev_error = error
    
        return control

    def follow_wall(self, robot_linearvelocity, set_point, right=False):
        if right:
            direction_coeff = -1
        else:
            direction_coeff = 1
        

        if(min(self.prox_sensors.get_value(1), 
               self.prox_sensors.get_value(2),
               self.prox_sensors.get_value(3),
               self.prox_sensors.get_value(4),
               self.prox_sensors.get_value(5),
               self.prox_sensors.get_value(6)) < set_point):
            self.set_velocity(robot_linearvelocity/3, -0.22*direction_coeff)
        else:
            if not right: 
                wall_dist = min(self.prox_sensors.get_value(1),
                                self.prox_sensors.get_value(0))
            else:
                wall_dist = min(self.prox_sensors.get_value(7),
                                self.prox_sensors.get_value(8))

            if (wall_dist < self.prox_sensors.max_range):
                error = wall_dist - set_point
                control = self.pid(error)
                self.set_velocity(robot_linearvelocity, control*direction_coeff)
            else:
                self.set_velocity(robot_linearvelocity, 0.08*direction_coeff)
        self.state = MoveState.FOLLOW_WALL
                
       
#("test")
    
    
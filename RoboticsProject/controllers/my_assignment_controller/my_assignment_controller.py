#Viraj Patel
#201351707


from controller import Supervisor
import occupancy_grid as ogrid
import pioneer_navx as pn
import pioneer_proxsensors as pps
import math
import pose
from controller import Robot
from pioneer_navx import MoveState

#def getTime(self):

def run_robot(robot):


        
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    camera = robot.getDevice('camera') # translate y=0.21
    if camera is not None:
        camera.enable(timestep)
        
        
        
#############################################################################
   # nav = pn.PioneerNavX(robot)
   # robot_pose = pose.Pose(0.0, 0.0, 0.0)

    my_pose = pose.Pose(0.0, 0.0, 0.0) #nav.get_real_pose() 
    prox_sensors = pps.PioneerProxSensors(robot, "sensor_display", my_pose);
    nav = pn.PioneerNavX(robot,my_pose,prox_sensors)

##############################################################################   
    
    time_elapsed = 0
    target_time = 0
    robot_velocity = 0.3
    end_time = 0
    
    # 2nd argument determines how many cells per meter of the arena.
    # Use 20 for testing navigation, but 50 for high-quality map (slow)
    occupancy_grid = ogrid.OccupancyGrid(robot, 1, "occupancy_grid_display", my_pose, prox_sensors);
    
    # define schedule
    schedule = [ MoveState.FOLLOW_WALL]        
    state = schedule[0]  # current state
    schedule_index = -1
    while robot.step(timestep) != -1:
        state = nav.state
        #print (time_elapsed)
    
        my_pose = nav.get_real_pose()
        prox_sensors.set_pose(my_pose)
        prox_sensors.paint()
        

        occupancy_grid.set_pose(my_pose)
        occupancy_grid.map()
        occupancy_grid.paint()
        if (time_elapsed > target_time):
            time_elapsed = 0
            timer = 0
            # select next action in schedule if not stopped
            schedule_index = (schedule_index +1) % len(schedule)
            nav.state = schedule[schedule_index]
            
            if (nav.state == MoveState.FORWARD):
                target_time = nav.forward(0.8, robot_velocity)
            elif (nav.state == MoveState.ARC):
                target_time = nav.arc(math.pi/2.0, 0.0, robot_velocity)
            elif (nav.state == MoveState.STOP):
                nav.stop()
                target_time = 60 * 1000
            
            
            elif (nav.state == MoveState.FOLLOW_WALL):
                target_time = 0
                nav.follow_wall(robot_velocity, 0.252, False)
                end_time = end_time + 1
                
                print (end_time)
                if (end_time == 2207):
                    print("STOP")
                    nav.state = MoveState.STOP
                    schedule = [MoveState.STOP]
                    
                    
                    

                #nav.follow_wall(robot_velocity, 0.252, False)
             
             

        else:
            time_elapsed += timestep    # Increment by the time state

            

if __name__ == "__main__":
    # create the Supervised Robot instance.
    my_robot = Supervisor()
    run_robot(my_robot)

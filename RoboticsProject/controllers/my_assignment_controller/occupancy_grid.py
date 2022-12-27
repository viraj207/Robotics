
import math
import pioneer_proxsensors as pps

import pose


class OccupancyGrid:
    """ A custom class to model an occupancy grid with optional display """

    # define the display constants
    DARKGRAY = 0x3C3C3C
    GRAY = 0xABABAB
    BLACK = 0x000000
    WHITE = 0xFFFFFF
        
    # Fixed log odds values (found empirically)  
    lprior = math.log(0.5/(1-0.5))
    locc = math.log(0.95/(1-0.95))
    lfree = math.log(0.45/(1-0.45))	

    HALFALPHA = 0.02               # Thickness of any wall found
    HALFBETA = math.pi/36.0          # sensor cone opening angle 



    def __init__(self, robot, grid_scale, display_name, robot_pose, prox_sensors):
        
        self.robot = robot
        self.robot_pose = robot_pose
        self.prox_sensors = prox_sensors
        self.radius = self.prox_sensors.get_radius()
        
        # Store Arena state instance variables
        arena = robot.getFromDef("ARENA")
        floorSize_field = arena.getField("floorSize")
        floorSize = floorSize_field.getSFVec2f()
        self.arena_width = floorSize[0]
        self.arena_height = floorSize[1]

        # ---------------------------------------------------------------------------
        # Initialise grid - grid_scale cells per m
        self.num_row_cells = int(grid_scale * self.arena_width)
        self.num_col_cells = int(grid_scale * self.arena_height)
        print(f"Buidling an Occupancy Grid Map of size {self.num_row_cells} x {self.num_col_cells}")

        self.grid = [self.lprior]*(self.num_row_cells * self.num_col_cells)

        #self.loop_count = 0
            
        # ------------------------------------------
        # If provided, set up the display
        self.display = robot.getDevice(display_name)
        if self.display is not None:
            self.device_width = self.display.getWidth()
            self.device_height = self.display.getHeight()
            
            #Determine the rendering scale factor           
            wsf = self.device_width / self.arena_width
            hsf = self.device_height / self.arena_height
            self.scalefactor = min(wsf, hsf)
            
            self.cell_width = int(self.device_width / self.num_row_cells)
            self.cell_height = int(self.device_height / self.num_col_cells)
        else:
            self.device_width = 0
            self.device_height = 0
            self.scalefactor = 0.0

    # ================================================================================== 
    # The following can be used externally to check the status of the grid map,
    # for example, to develop an exploration strategy.  
    def get_num_row_cells(self):
        return self.num_row_cells    
    def get_num_col_cells(self):
        return self.num_col_cells
    def get_grid_size():
        return len(self.grid)
    def get_cell_probability(self, i):
        return cell_probability(self.grid[i])

    # ================================================================================== 
    # helper methods for mapping to the display
    # Map the real coordinates to screen coordinates assuming
    #   the origin is in the center and y axis is inverted

    def scale(self, l):
        return int(l * self.scalefactor)
    def mapx(self, x):
        return int((self.device_width / 2.0) + self.scale(x))
    def mapy(self, y):
        return int((self.device_height / 2.0) - self.scale(y))

    def set_pose(self, p):
        self.robot_pose.set_pose_position(p)
        
    #Convert log odds into a probability
    def cell_probability(self, lodds):
        return 1 - (1 / (1 + math.exp(lodds)))
        
    
    # Get log odds value for cell x,y given the current pose
    def inv_sensor_model(self, p, x, y):
        dx = x - p.x
        dy = y - p.y
        r = math.sqrt(dx**2 + dy**2)                  # range
        phi = math.atan2(dy, dx) - p.theta          # bearing
        lo = self.lprior                                 # default return value
        
        # Remove distance from robot center to sensor.  If test fails then cell
        # center is within the robot radius so just reset to zero
        if (r > self.radius):
            r = r - self.radius
        else:
            r = 0.0
        
        # find nearest sensor to cell
        # initialise the angle to pi as other angles will be less (clockwise or anticlockwise)
        k_min_delta = math.pi
        k = 0
        for j in range(len(self.prox_sensors.ps)):
            kd = abs(self.prox_sensors.ps_pose[j].get_dtheta(phi))
            if (kd < k_min_delta):
                k = j
                k_min_delta = kd

        # we now known that k=closest sensor, and k_min_delta is the difference in angle
        z = self.prox_sensors.get_value(k)
        if (z == self.prox_sensors.max_range):
            lo = self.lprior
        elif ((r > min(self.prox_sensors.max_range, z + self.HALFALPHA)) or (k_min_delta > self.HALFBETA)):
            lo = self.lprior      # region 3        print("InvSensorModel for ({},{}), k={}, z={}, lo={}".format(x, y, k, z, lo))
        elif ((z < self.prox_sensors.max_range) and (abs(r-z) < self.HALFALPHA)):
            lo = self.locc        # region 1
        elif (r <= z):
            lo = self.lfree       # region 2
        
        return lo


    def update(self, new_pose=None):
        if new_pose is not None:
            self.robot_pose = new_pose
 
        # Manage refresh rate
        self.loop_count += 1
        
        if ((self.loop_count % 2) == 0):
            self.map()

        if ((self.loop_count % 2) == 0):
            self.paint()

    
    def map(self):
        x_orig_offset = self.arena_width / 2
        y_orig_offset = self.arena_height / 2
        
        x_inc = self.arena_width / self.num_row_cells
        y_inc = self.arena_height / self.num_col_cells
        
        x_cell_offset = x_inc / 2
        y_cell_offset = y_inc / 2
        
        for i in range(len(self.grid)):
            # Convert cell into a coordinate.  Recall that the arena is dimensions -n..+n
            x = x_inc * int(i % self.num_row_cells) - x_orig_offset + x_cell_offset
            y = -(y_inc * int(i / self.num_row_cells) - y_orig_offset + y_cell_offset)

            self.grid[i] = self.grid[i] + self.inv_sensor_model(self.robot_pose, x, y) - self.lprior


    def paint(self):

        if self.display is None:
            return

        # draw a backgound
        self.display.setColor(0xF0F0F0)
        self.display.fillRectangle(0, 0, self.device_width, self.device_height)
        
        # draw values for occupancy grid  
        self.coverage = 0.0  
        for i in range(len(self.grid)):
            p = self.cell_probability(self.grid[i])
            x = self.cell_width * int(i % self.num_row_cells)
            y = self.cell_height * int(i / self.num_col_cells)
                        
            if (p < 0.1):
                self.display.setColor(self.WHITE)
            elif (p < 0.2):
                self.display.setColor(0xDDDDDD)
            elif (p < 0.3):
                self.display.setColor(0xBBBBBB)
            elif (p < 0.4):
                self.display.setColor(0x999999)
            elif (p > 0.9):
                self.display.setColor(self.BLACK)
            elif (p > 0.8):
                self.display.setColor(0x222222)
            elif (p > 0.7):
                self.display.setColor(0x444444)
            elif (p > 0.6):
                self.display.setColor(0x666666)
            else:
                self.display.setColor(self.GRAY)
                
            self.display.fillRectangle(x, y, self.cell_width, self.cell_height)
            
            if(p < 0.1) or (p > 0.9):
                self.coverage += 1.0
        
        # normalise coverage
        self.coverage = self.coverage / len(self.grid)
        
        self.display.setColor(self.GRAY)
        # vertical lines
        x=0
        for i in range(self.num_row_cells):
            self.display.drawLine(x, 0, x, self.device_height)
            x += self.cell_width

        # horizontal lines
        y=0
        for j in range(self.num_row_cells):
            self.display.drawLine(0, y, self.device_height, y)
            y += self.cell_height
                
        # draw robot body
        self.display.setColor(self.WHITE)
        self.display.fillOval(self.mapx(self.robot_pose.x),
                              self.mapy(self.robot_pose.y),
                              self.scale(self.radius),
                              self.scale(self.radius))
                              
        self.display.setColor(self.DARKGRAY)
        self.display.drawOval(self.mapx(self.robot_pose.x),
                              self.mapy(self.robot_pose.y),
                              self.scale(self.radius),
                              self.scale(self.radius))
        self.display.drawLine(self.mapx(self.robot_pose.x),
                              self.mapy(self.robot_pose.y),
                              self.mapx(self.robot_pose.x + math.cos(self.robot_pose.theta) * self.radius),
                              self.mapy(self.robot_pose.y + math.sin(self.robot_pose.theta) * self.radius))                             
        
        # Provide coverage percentage
        self.display.setColor(0xF0F0F0)  # Off White
        self.display.fillRectangle(self.device_width-80, self.device_height-18, self.device_width-20, self.device_height)
        self.display.setColor(0x000000)  # Black
        self.display.drawRectangle(self.device_width-80, self.device_height-18, self.device_width-20, self.device_height)


        self.display.setFont("Arial", 10, True)
        self.display.drawText(f"{self.coverage * 100:.2f}%", self.device_width-60, self.device_height-14);

 
from controller import Robot
import math
import AStar as astar
import WebotsNavAlgorithms as nav

#Sources used for bits:
 #LIADR: https://github.com/cyberbotics/webots/blob/master/projects/samples/devices/controllers/lidar/lidar.py
 #LED: https://github.com/cyberbotics/webots/blob/master/projects/samples/devices/controllers/led/led.py
 #GPS: https://www.youtube.com/watch?v=3nDkEnxInrs
 #LIDAR 2:https://www.youtube.com/watch?v=7A9h7d-XWSA&t=277s
 #PID: https://www.youtube.com/watch?v=qexnI4-Cdxk&t=32s
 #Some python stuff:https://www.w3schools.com/python/ref_math_isnan.asp

#--------------------------- Load the Grid + set Goal Location Targets
filemap = "SampleWorld.csv"
goal_X = 2.0          
goal_Y = -6.0        
goal_THETA = -5.0   #0 

#------------------------- sanity corrections 
TIME_STEP = 64                
MAX_SPEED = 6.28               
GRID_RES = 0.1         
buffer_width = 0     
WAYPOINT_TOL = GRID_RES * 1 
final_pos_corr = 0.5  # pos good up to +- 0.5 err    
final_ang_err = 2.0        

#------------------------------- PID values (just P)
KP = 0.002
KI = 0.0
KD = 0.0

#----------------------------------- avoidance  + clearence
OBSTACLE_THRESHOLD = 0.74  # 0.7 - 0.6 = 0.1 room
AVOID_SPEED = 0.8 * MAX_SPEED  

#------------------------------------- LED codes
LED_WAIT = 4 #Yellow for Hold and check theta goal
LED_OK = 3; #Green for Done
LED_WP = 2; #Blue for moving
LED_ALERT = 1 # Red for object detected + avoied



#----------------------------------- Call all the stuff we need
def init_devices():
    robot = Robot()
    
    left = robot.getDevice('left_motor')
    right = robot.getDevice('right_motor')
    for m in (left, right):
        m.setPosition(float('inf'))
        m.setVelocity(0.0)
          
    gps_f = robot.getDevice('gps_front'); gps_f.enable(TIME_STEP)
    gps_b = robot.getDevice('gps_back');  gps_b.enable(TIME_STEP)
    
    lidar = robot.getDevice('lidar'); lidar.enable(TIME_STEP); lidar.enablePointCloud()
    
    leds = [robot.getDevice(f"led_{i}") for i in range(1, 5)]
    return robot, left, right, gps_f, gps_b, lidar, leds
    
    #print(f": {idx}") debug each var by itself


#--------------------------------------- GPS Values
def get_pose(gps_f, gps_b):
    pf = gps_f.getValues()  
    pb = gps_b.getValues()
    x = (pf[0] + pb[0]) / 2  #[X,Y]
    y = (pf[1] + pb[1]) / 2
    theta = math.atan2(pf[1] - pb[1], pf[0] - pb[0])
    return x, y, theta

#-------------------------------------- LIDAR Setup
def obstacle_ahead(lidar):
    ranges = lidar.getRangeImage()
    n = len(ranges)
    front = [r for r in ranges[n//3:2*n//3] if math.isfinite(r)]
    return (min(front) if front else float('inf')) < OBSTACLE_THRESHOLD


#------------------------------------- LED State setup 
def set_leds(leds, code):
    for led in leds:
        led.set(code) # 1 = Red, 2 = Grenn blah blah

#------------------------------------- A* stuff
def load_map():
    grid = nav.readOccupancyGrid(filemap)
    return nav.addBufferToOccupancyGrid(grid, buffer_width)

def plan_path(x, y, grid):
    return astar.aStar(x, y, goal_X, goal_Y, grid)

#------------------------------------- PID thoery
prev_error = 0.0
integral = 0.0 # ignore

def pid_control(error, dt):
    global prev_error ,integral
    integral += error * dt
    derivative = (error - prev_error) / dt
    prev_error = error
    return KP * error

def pid_reset():
    global prev_error ,integral
    prev_error = 0.0
    integral = 0.0


#------------------------ Main Logic Loop
def follow_path(robot, left, right, gps_f, gps_b, lidar, leds, path):
    idx = 0 #track waypoint (Im using the GPS link from github)
    dt = TIME_STEP / 1000.0
    mode = "FOLLOW"  #Normal path

    while robot.step(TIME_STEP) != -1:
        x, y, theta = get_pose(gps_f, gps_b)
        x, y, theta = get_pose(gps_front, gps_back)
        #print(f"X: {x:.2f} m, Y: {y:.2f} m, Thetaa: {math.degrees(theta):.1f}°")


        #------------------Spot check (Find Hypot based on GPS)
        goal_dist = math.hypot(goal_X - x, goal_Y - y)
        if mode == "FINAL_ALIGN" or (idx >= len(path) and goal_dist < final_pos_corr):
            #--------------- Find the angle (correct)
            desired_theta = math.radians(goal_THETA % 360.0)
            ang_err = nav.angleToTarget(theta, desired_theta)
            ang_err_deg = abs(math.degrees(ang_err))
            if goal_dist < final_pos_err and ang_err_deg < final_ang_err:
                set_leds(leds, LED_OK) #GREEN
                left.setVelocity(0); right.setVelocity(0)
                print("Goal reached !!")
                return
            #-------------------- rotate in place + LED Yellow
            set_leds(leds, LED_WAIT) 
            pid_out = pid_control(ang_err, dt) # P math to fix angle
            ls = max(min(-pid_out, MAX_SPEED), -MAX_SPEED)
            rs = max(min(+pid_out, MAX_SPEED), -MAX_SPEED)
            left.setVelocity(ls); right.setVelocity(rs)
            mode = "FINAL_ALIGN"
            continue

        #--------------------- Orintation check
        if idx >= len(path):
            mode = "FINAL_ALIGN"
            pid_reset()
            continue

        wp_x, wp_y = path[idx] # Read current waypoints (same as HYP above)
        dist = math.hypot(wp_x - x, wp_y - y)

        #---------------------------------Optimization
        if idx < len(path) - 1:
            nxt_x, nxt_y = path[idx+1]
            dist_next = math.hypot(nxt_x - x, nxt_y - y) #Might be the issue
            if dist_next + 0.05 < dist:
                idx += 1
                continue

        #-------------------------------Made it to point x check
        if dist < WAYPOINT_TOL:
            set_leds(leds, LED_OK)
            print(f"Point: {idx}")
            idx += 1
            pid_reset() 
            continue

        #--------------------------------LIDAR read (if you see something use the turn speed)
        if obstacle_ahead(lidar):
            set_leds(leds, LED_ALERT)
            left.setVelocity(-AVOID_SPEED)
            right.setVelocity(AVOID_SPEED)
            continue

        #----------------- P correction
        desired = nav.getTargetPose(wp_x, wp_y, x, y)
        alpha = nav.angleToTarget(theta, desired)
        alpha = math.atan2(math.sin(alpha), math.cos(alpha)) #Obejct to robot angle
        correction = pid_control(alpha, dt)
        base = 0.1 * MAX_SPEED
        ls = max(min(base - correction, MAX_SPEED), -MAX_SPEED)
        rs = max(min(base + correction, MAX_SPEED), -MAX_SPEED)
        set_leds(leds, LED_OK) #DONE
        left.setVelocity(ls)
        right.setVelocity(rs)


robot, left_motor, right_motor, gps_front, gps_back, lidar, leds = init_devices()

# ---------------------- Acquire initial GPS coordinates and robot pose
set_leds(leds, LED_WAIT)
while robot.step(TIME_STEP) != -1:
    x, y, theta = get_pose(gps_front, gps_back)
    if not (math.isnan(x) or math.isnan(y)): #random but works
        set_leds(leds, LED_OK)
        break

# -------------------- Load map and debug start/goal (use the SAME plane as get_pose)
grid = load_map()
x, y, _ = get_pose(gps_front, gps_back)
start_r = int(y / GRID_RES)
start_c = int(x / GRID_RES)
print(f"current → grid[{start_r}][{start_c}] = {grid[start_r][start_c]}")
goal_r = int(goal_Y / GRID_RES)
goal_c = int(goal_X / GRID_RES)
print(f"end  → grid[{goal_r}][{goal_c}] = {grid[goal_r][goal_c]}")
if grid[start_r][start_c] != '0' or grid[goal_r][goal_c] != '0':
    set_leds(leds, LED_ALERT)
    print("GPS Fail!!!! check co-ordinates")
    exit()

# -------------------------------------- Plan and follow
path, ok = plan_path(x, y, grid)
if not ok:
    set_leds(leds, LED_ALERT)
    print("Grid Fail! Move Robot")
else:
    print("planned path:")
    for i, (wx, wy) in enumerate(path):
        print(f"  {i}: ({wx:.2f}, {wy:.2f})")
    follow_path(robot, left_motor, right_motor,
                     gps_front, gps_back, lidar, leds, path)
    print("Navigation Wroks!")


#keyboard ? 


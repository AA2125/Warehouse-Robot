from controller import Robot
import math
#import AStar as astar
#import WebotsNavAlgorithms as nav

#GPS: https://www.youtube.com/watch?v=3nDkEnxInrs

#----------------------- Goal
#goal_X = 2.0          
#goal_Y = -6.0        
#goal_THETA = -5.0   #0 

#------------------------- intial bits
robot = Robot()
TIME_STEP = 64                
MAX_SPEED = 6.28               

#-------------------------Motors
left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor')
for m in (left_motor, right_motor):
    m.setPosition(float('inf'))
    m.setVelocity(0.0)
    
#--------------------------GPS call          
gps_f = robot.getDevice('gps_front'); gps_f.enable(TIME_STEP)
gps_b = robot.getDevice('gps_back');  gps_b.enable(TIME_STEP)

#--------------------------------------- GPS Values
def get_pose(gps_f, gps_b):
    pf = gps_f.getValues()  
    pb = gps_b.getValues()
    x = (pf[0] + pb[0]) / 2  #[X,Y]
    y = (pf[1] + pb[1]) / 2
    theta = math.atan2(pf[1] - pb[1], pf[0] - pb[0])
    return x, y, theta
 
while robot.step(TIME_STEP) != -1:
       x, y, theta = get_pose(gps_f, gps_b)
       print(f"X: {x:.2f} m, Y: {y:.2f} m, Thetaa: {math.degrees(theta):.1f}°")

#add +170 to offset the theta
left_motor.setVelocity(0.2 * MAX_SPEED)
right_motor.setVelocity(0.2 * MAX_SPEED)

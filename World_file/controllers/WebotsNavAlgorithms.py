#! /bin/python3.10

"""! @brief Supporting material for the 2023/24 4B12 individual project."""
##
# @file WebotsNavAlgorithms.py
#
# @brief Provies a varity of functions which may be useful while completing the 4B12 individual project.
#
# @section author_nav_algo Author(s)
# - Created by Patrick Lynch.

import math
import numpy as np

def readOccupancyGrid(filename):
    """! Reads an occupancy grid from a csv file, creates and returns the occupancy grid as a 2D python list.
    @param filename **string**: address pointing to the location of the occupancy grid csv file on your computer, i.e. "../OccupancyGrid.csv"
    @return **list**: The occupancy grid from the csv file indicated, in the form of a 2D python list
    """
    delimiter = ','
    occupancy_grid = np.genfromtxt(filename, delimiter=delimiter, dtype=str)
    occupancy_grid[occupancy_grid == ''] = '0'
    occupancy_grid = np.char.strip(occupancy_grid, "'")
    return occupancy_grid

def printOccupancyGrid(occupancy_grid):
    """! Exhaustively prints all cells in a given occupancy grid, this is useful for debugging
    @param occupancy_grid **list**: A 2D occupancy grid in the form of a 2D python list, i.e. [[1, 1, 1, 1], [1, 0, 0, 1], [1, 0, 0, 1], [1, 1, 1, 1]]
    @returns **NULL**: This function doesn't return anything 
    """
    for row in occupancy_grid:
        for element in row:
            if type(element) != np.str_:
                print(type(element))
            print(element, end='')
            print(" ", end='')
        print()

def addBufferToOccupancyGrid(occupancy_grid, buffer_width):
    """! Takes an existing occupancy grid and inflates the size of each of the obsticals in the map, i.e. make them appear larger than they are.
    @param occupancy_grid **list**: A 2D occupancy grid in the form of a 2D python list, i.e. [[1, 1, 1, 1], [1, 0, 0, 1], [1, 0, 0, 1], [1, 1, 1, 1]]
    @param buffer_width **int**: The number of cells which you want to 'inflate' your obsticales by, i.e. for any occupied cell mark all neighbouring cells within <buffer_size> cells as occupied also.
    @return **list**: An updated occupancy grid in the form of a 2D python list.
    """   

    # Get the global dimensions of the occupancy grid
    rows, cols = occupancy_grid.shape

    # Create a modified occupancy grid
    occupancy_grid_buff = np.zeros((rows, cols), dtype=str)

    for row_index in range(rows):
       for column_index in range(cols):
           occupancy_grid_buff[row_index][column_index] = str(0)

    for row_index in range(rows):
        for column_index in range(cols):
            # If the cell is occupied, mark neighbouring cells as occupied also
            if int(occupancy_grid[row_index, column_index]) == 1:
                # Update the current cell and its neighbors in the new modified occupancy grid
                for row_index_inc in range(-1*buffer_width, buffer_width+1):
                    for column_index_inc in range(-1*buffer_width, buffer_width+1):
                        # Check if the cell is within the bounds of the occupancy grid
                        if 0 <= row_index+row_index_inc < rows and 0 <= column_index+column_index_inc < cols:
                            occupancy_grid_buff[row_index+row_index_inc, column_index+column_index_inc] = str(1)
    
    return occupancy_grid_buff

def exportOccupancyGridAsCsv(occupancy_grid):
    """! Takes an existing occupancy grid and exports the information within it as a csv file.
    @param occupancy_grid **list**: A 2D occupancy grid in the form of a 2D python list, i.e. [[1, 1, 1, 1], [1, 0, 0, 1], [1, 0, 0, 1], [1, 1, 1, 1]]
    @returns **NULL**: This function doesn't return anything
    """
    f = open("occupancy_grid.csv", "a")

    for row in occupancy_grid:
        for column in row:
            f.write(str(column))
            f.write(",")
        f.write("\n")
    f.close()

def angleToTarget(robot_theta, target_theta):
    """! Take the current robot pose and the desired robot pose, i.e. the one which would point the robot directly at the next waypoint, and returns the difference, i.e. the angle the robot needs to turn to achieve its desired position.
    @param robot_theta **float**: The current angle of the robot relative to the global cartesian coordinate frame
    @param target_theta **float**: The desired angle of the robot relative to the global cartesian coordinate frame
    @return **float**: Returns an angle value which represents the difference between the angles indicated by the input parameters
    """
    delta_theta = target_theta - robot_theta
    if delta_theta > math.radians(180):
        delta_theta -= math.radians(360)
    elif delta_theta < math.radians(-180):
        delta_theta += math.radians(360)
    return delta_theta

def getTargetPose(goal_x, goal_y, robot_x, robot_y):
    """! Takes a goal/waypoint position and the robot's position and calculates the desired 'theta' of the robot so that it will be 'facing'/'pointing towards' the goal/waypoint
    @param goal_x **float**: The x coordinate of the goal/next waypoint relative to the global cartesian coordinate frame
    @param goal_y **float**: The y coordinate of the goal/next waypoint relative to the global cartesian coordinate frame
    @param robot_x **float**: The current x coordinate of the robot relative to the global cartesian coordinate frame
    @param robot_y **float**: The current y coordinate of the robot relative to the global cartesian coordinate frame
    @return **float**: An angle relative to the global cartesian coordinate frame
    """
    goal = [goal_x, goal_y]
    robot_location = [robot_x, robot_y]
    target_orientation_vector = [goal[0]-robot_location[0], goal[1]-robot_location[1]]
    mag_target_orientation_vector = math.sqrt (pow(target_orientation_vector[0],2)+pow(target_orientation_vector[1],2))
    target_orientation_unit_vector = [0, 0]
    
    if mag_target_orientation_vector != 0:
        target_orientation_unit_vector[0] = target_orientation_vector[0]/mag_target_orientation_vector
        target_orientation_unit_vector[1] = target_orientation_vector[1]/mag_target_orientation_vector

        target_pose = -1*(math.asin(target_orientation_unit_vector[1]))

        if (target_orientation_unit_vector[0] < 0 and target_orientation_unit_vector[1] > 0):
            target_pose = (-1*math.pi) - target_pose
        elif (target_orientation_unit_vector[0] < 0 and target_orientation_unit_vector[1] < 0):
            target_pose = math.pi-target_pose
    
    return target_pose
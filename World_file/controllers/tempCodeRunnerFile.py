"! Takes a start position, end position and an occupancy grid and returns a list of waypoints leading from the start position to the end position
    @param robot_x **float**: The x coordinate of the start position relative to the global cartesian coordinate frame
    @param robot_y **float**: The y coordinate of the start position relative to the global cartesian coordinate frame
    @param goal_x **float**: The x coordinate of the end position relative to the global cartesian coordinate frame
    @param goal_y **float**: The y coordinate of the end position relative to the global cartesian coordinate frame
    @param occupancy_grid **list**: A 2D occupancy grid in the form of a 2D python list, i.e. [[1, 1, 1, 1], [1, 0, 0, 1], [1, 0, 0, 1], [1, 1, 1, 1]]
    @return **list**: A list of waypoints which leads from the start position to the end position i.e. [[1, 2], [2, 2], [3, 3], [4, 3]]
    @return **boolean**: Indicator of the status of the search for a solution, i.e. did it successfully
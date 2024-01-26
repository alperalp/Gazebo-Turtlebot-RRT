# RRT Algorithm in Gazebo Using Turtlebot3 with ROS

## A basic project about path planning with rrt and simulation in Gazebo enviroment

This is a project I created for my Autonomous Systems class. It involves planning a path using the RRT algorithm on a provided map. Additionally, you can utilize a generated Gazebo map for simulation.

## Installation and Usage
1. Clone the package into your working enviroment. Also make sure that you have turtlebot3 packages from robotis "https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup"
2. Run the rrt planning script -```rosrun my_rrt_pkg rrt_planner.py```. This will generate a world file named "rrt_world.world" and a text file with cordinates for route named "rrt_nodes.txt"

![rrt_route](https://github.com/alperalp/Gazebo-Turtlebot-RRT/assets/58988396/5a3552d0-ff72-4002-8916-44a69767f823)

3. Launch the simulation using ```roslaunch my_rrt_pkg gazebo_launch.launch```. This will boot gazebo simulation with our map and with a turtlebot3

![gazebo](https://github.com/alperalp/Gazebo-Turtlebot-RRT/assets/58988396/32f66903-63de-475d-9416-a4d7752e08a0)

4. Run the movement script to start the robot using ```rosrun my_rrt_pkg move_robot.py```. This will take the "rrt_nodes.txt" in project folder but you can give another file as argument if you want.
5. (Optional) You can launch the rviz to visualize the simulation using ```roslaunch turtlebot3_navigation turtlebot3_navigation.launch```

![sim_route](https://github.com/alperalp/Gazebo-Turtlebot-RRT/assets/58988396/e093e688-9a23-4fed-8c3a-b9c892799887)


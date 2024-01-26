#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import random

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def is_collision_free(x, y, obstacles, safety_distance=0.7):
    for obstacle in obstacles:
        if (obstacle[0] - safety_distance) < x < (obstacle[1] + safety_distance) and \
           (obstacle[2] - safety_distance) < y < (obstacle[3] + safety_distance):
            return False  # Collision detected
    return True  # Collision-free

def generate_random_node(width, height):
    return Node(random.uniform(0, width), random.uniform(0, height))

def nearest_neighbor(random_node, nodes):
    return min(nodes, key=lambda node: np.hypot(node.x - random_node.x, node.y - random_node.y))

def steer(from_node, to_node, step_size, obstacles):
    distance = np.hypot(to_node.x - from_node.x, to_node.y - from_node.y)
    
    if distance < step_size:
        return to_node
    else:
        theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + step_size * np.cos(theta)
        new_y = from_node.y + step_size * np.sin(theta)
        
        # Check if the new node is collision-free and at a safe distance from obstacles
        if is_collision_free(new_x, new_y, obstacles):
            return Node(new_x, new_y)
        else:
            return from_node

def rrt(width, height, obstacles, start, goal, iterations=1000, step_size=0.5, max_attempts=100):
    nodes = [start]
    
    for _ in range(iterations):
        random_node = generate_random_node(width, height)
        
        if not is_collision_free(random_node.x, random_node.y, obstacles):
            continue  # Skip if collision
        
        nearest_node = nearest_neighbor(random_node, nodes)
        attempts = 0
        while attempts < max_attempts:
            new_node = steer(nearest_node, random_node, step_size, obstacles)
            if new_node != nearest_node:
                break
            attempts += 1
        
        if attempts >= max_attempts:
            continue  # Skip if unable to find a valid steering node
        
        if is_collision_free(new_node.x, new_node.y, obstacles):
            new_node.parent = nearest_node
            nodes.append(new_node)
        
        # Check if goal is reached
        if np.hypot(new_node.x - goal.x, new_node.y - goal.y) < step_size:
            goal.parent = new_node
            nodes.append(goal)
            
            # Extract the selected path
            selected_path = [goal]
            current = goal
            while current.parent:
                selected_path.append(current.parent)
                current = current.parent
            selected_path = selected_path[::-1]  # Reverse the path to start from the beginning
            
            return selected_path  # Return only the correct path
    
    return []  # Return an empty list if no valid path is found

def visualize_rrt(nodes, obstacles, start, goal):
    plt.figure(figsize=(6, 6))
    
    # Plot obstacles
    for obstacle in obstacles:
        plt.plot([obstacle[0], obstacle[1], obstacle[1], obstacle[0], obstacle[0]],
                 [obstacle[2], obstacle[2], obstacle[3], obstacle[3], obstacle[2]], 'k-')
    
    # Plot edges in the tree
    for node in nodes:
        if node.parent:
            plt.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b-')
    
    # Plot start and goal
    plt.plot(start.x, start.y, 'go', markersize=10)
    plt.plot(goal.x, goal.y, 'ro', markersize=10)
    
    # Highlight the path
    path_x = []
    path_y = []
    current = goal
    while current.parent:
        path_x.append(current.x)
        path_y.append(current.y)
        current = current.parent
    path_x.append(start.x)
    path_y.append(start.y)
    plt.plot(path_x, path_y, 'g-', linewidth=2)
    
    plt.title('RRT with Obstacles (Avoiding Close Proximity)')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.grid(True)
    plt.show()
def create_gazebo_world(width, height, obstacles, world_filename='worlds/rrt_world.world', nodes_filename='rrt_nodes.txt'):

    with open(nodes_filename, 'w') as nodes_file:
        for node in nodes:
            nodes_file.write(f'{node.x} {node.y}\n')
    
    with open(world_filename, 'w') as world_file:
        world_file.write('<?xml version="1.0" ?>\n')
        world_file.write('<sdf version="1.6">\n')
        world_file.write('<world name="default">\n')

        # Include the ground plane
        world_file.write('\t<include>\n')
        world_file.write('\t\t<uri>model://ground_plane</uri>\n')
        world_file.write('\t</include>\n')

        # Add walls around the map
        wall_thickness = 1.0  # You can adjust the thickness of the walls
        walls = [
            [-1.5, 11.5, -1.5, wall_thickness - 1.5],  # Bottom wall
            [-1.5, wall_thickness -1.5, -1.5, 11.5],  # Left wall
            [11.5 - wall_thickness, 11.5, -1.5, 11.5],  # Right wall
            [-1.5, 11.5, 11.5 - wall_thickness, 11.5],  # Top wall
        ]

        for i, wall in enumerate(walls):
            world_file.write(f'\t<model name="wall_{i}">\n')
            world_file.write('\t\t<static>true</static>\n')
            world_file.write('\t\t<link name="link">\n')
            world_file.write('\t\t\t<collision name="collision">\n')
            world_file.write('\t\t\t\t<geometry>\n')
            world_file.write('\t\t\t\t\t<box>\n')
            world_file.write(f'\t\t\t\t\t\t<size>{wall[1] - wall[0]} {wall[3] - wall[2]} 1.0</size>\n')
            world_file.write('\t\t\t\t\t</box>\n')
            world_file.write('\t\t\t\t</geometry>\n')
            world_file.write('\t\t\t</collision>\n')
            world_file.write('\t\t\t<visual name="visual">\n')
            world_file.write('\t\t\t\t<geometry>\n')
            world_file.write('\t\t\t\t\t<box>\n')
            world_file.write(f'\t\t\t\t\t\t<size>{wall[1] - wall[0]} {wall[3] - wall[2]} 1.0</size>\n')
            world_file.write('\t\t\t\t\t</box>\n')
            world_file.write('\t\t\t\t</geometry>\n')
            world_file.write('\t\t\t</visual>\n')
            world_file.write('\t\t</link>\n')
            world_file.write('\t\t<pose>{:.2f} {:.2f} 0.0 0 0 0</pose>\n'.format(
                (wall[0] + wall[1]) / 2, (wall[2] + wall[3]) / 2))
            world_file.write('\t</model>\n')

        # Add obstacles to the world
        for i, obstacle in enumerate(obstacles):
            world_file.write(f'\t<model name="obstacle_{i}">\n')
            world_file.write('\t\t<static>true</static>\n')
            world_file.write('\t\t<link name="link">\n')
            
            # Collision
            world_file.write('\t\t\t<collision name="collision">\n')
            world_file.write('\t\t\t\t<geometry>\n')
            world_file.write('\t\t\t\t\t<box>\n')
            world_file.write(f'\t\t\t\t\t\t<size>{obstacle[1] - obstacle[0]} {obstacle[3] - obstacle[2]} 1.0</size>\n')
            world_file.write('\t\t\t\t\t</box>\n')
            world_file.write('\t\t\t\t</geometry>\n')
            world_file.write('\t\t\t</collision>\n')
            
            # Visual
            world_file.write('\t\t\t<visual name="visual">\n')
            world_file.write('\t\t\t\t<geometry>\n')
            world_file.write('\t\t\t\t\t<box>\n')
            world_file.write(f'\t\t\t\t\t\t<size>{obstacle[1] - obstacle[0]} {obstacle[3] - obstacle[2]} 1.0</size>\n')
            world_file.write('\t\t\t\t\t</box>\n')
            world_file.write('\t\t\t\t</geometry>\n')
            world_file.write('\t\t\t</visual>\n')
            
            world_file.write('\t\t</link>\n')
            world_file.write('\t\t<pose>{:.2f} {:.2f} 0.0 0 0 0</pose>\n'.format(
                (obstacle[0] + obstacle[1]) / 2, (obstacle[2] + obstacle[3]) / 2))
            world_file.write('\t</model>\n')

        # Add sunlight
        world_file.write('\t<light name="sun" type="directional">\n')
        world_file.write('\t\t<cast_shadows>true</cast_shadows>\n')
        world_file.write('\t\t<pose>0 0 10 0 0 0</pose>\n')
        world_file.write('\t\t<diffuse>0.8 0.8 0.8 1</diffuse>\n')
        world_file.write('\t\t<specular>0.2 0.2 0.2 1</specular>\n')
        world_file.write('\t\t<attenuation>\n')
        world_file.write('\t\t\t<range>1000</range>\n')
        world_file.write('\t\t\t<constant>0.9</constant>\n')
        world_file.write('\t\t\t<linear>0.01</linear>\n')
        world_file.write('\t\t\t<quadratic>0.001</quadratic>\n')
        world_file.write('\t\t</attenuation>\n')
        world_file.write('\t\t<direction>-0.5 0.5 -0.5</direction>\n')
        world_file.write('\t</light>\n')

        world_file.write('</world>\n')
        world_file.write('</sdf>\n')

    print(f'Gazebo world file "{world_filename}" created.')

if __name__ == "__main__":
    # Define workspace dimensions
    width, height = 10, 10
    
    # Define obstacles as [x_min, x_max, y_min, y_max]
    obstacles = [[1, 6, 2, 7], [8, 9, 1, 4]]
    
    # Define start and goal nodes
    start = Node(1, 1)
    goal = Node(9, 9)
    
    # Run RRT algorithm
    nodes = rrt(width, height, obstacles, start, goal, iterations=1000, step_size=2, max_attempts=100)
    
    # Visualize the RRT with highlighted path
    visualize_rrt(nodes, obstacles, start, goal)
    
    # Create Gazebo world file
    create_gazebo_world(width, height, obstacles)

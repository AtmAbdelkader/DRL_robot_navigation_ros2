########################################
#Dynamic Obstacle using ROS2 humble 
# Author : Belabed Abdelkader
########################################

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import numpy as np
import threading

class DynamicObstacleNode(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_node')
        self.get_logger().info("Dynamic Obstacle Node initialized")
        
        self.update_interval = 0.05  # time between updates in seconds
        self.box_speed = 0.8  # current movement speed (m/s)
        self.box_acceleration = 0.01  # Box acceleration (m/s^2)
        
        # Starting positions distributed in a grid-like pattern
        self.box_start_positions = {i: {'x': np.random.uniform(-8.7, 4.0),
                                        'y': np.random.uniform(0.6, 15.5),
                                        'z': 0.3} for i in range(9)}  # Start positions
        
        # Velocity for each box (change vx, vy to create varied movements)
        self.box_positions = {i: {'x': self.box_start_positions[i]['x'], 'y': self.box_start_positions[i]['y'],
                                'vx': np.random.uniform(-self.box_speed, self.box_speed), 
                                'vy': np.random.uniform(-self.box_speed, self.box_speed)} for i in range(9)}

        # limit environment
        self.x_min, self.x_max = -9.0, 4.5
        self.y_min, self.y_max = 1.0, 13.5

        # Create a client for the Gazebo SetEntityState service
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        if not self.set_state_client.wait_for_service(timeout_sec=10.0):  # Wait for 10 seconds
            self.get_logger().error("Service /gazebo/set_entity_state not available. Exiting.")
            raise RuntimeError("Service /gazebo/set_entity_state not available.")
        
        self.get_logger().info("Service /gazebo/set_entity_state is available.")

        # Start update thread
        self.update_thread = threading.Thread(target=self.update_box_positions)
        self.update_thread.daemon = True
        self.update_thread.start()
        self.get_logger().info("Update thread started")

    def update_box_positions(self):
        rate = self.create_rate(1 / self.update_interval)  # update by time period 
        safe_distance = 1.0 
        goal_x, goal_y = -2.5, 14.5
        x_r, y_r = -1.0, 1.5 

        while rclpy.ok():
            #self.get_logger().info("Updating box positions")
            for i in range(9):
                name = "cardboard_box_" + str(i)
                box_state = EntityState()
                box_state.name = name

                # Get box update
                pos = self.box_positions[i]
                x, y = pos['x'], pos['y']
                vx, vy = pos['vx'], pos['vy']
                
                # update velocity
                vx += self.box_acceleration * self.update_interval
                vy += self.box_acceleration * self.update_interval

                # update pose
                x += vx * self.update_interval
                y += vy * self.update_interval

                # check the proximity of the obstacle to the goal_point
                distance_to_goal = np.sqrt((x - goal_x) ** 2 + (y - goal_y) ** 2)
                if distance_to_goal < safe_distance:
                    # if the obstacle is too close to the goal_point, change direction 
                    x = np.random.uniform(self.x_min, self.x_max)
                    y = np.random.uniform(self.y_min, self.y_max)
                    vx = np.random.uniform(-self.box_speed, self.box_speed)
                    vy = np.random.uniform(-self.box_speed, self.box_speed)
                
                distance_to_robot = np.sqrt((x - x_r) ** 2 + (y - y_r) ** 2)
                if distance_to_robot < safe_distance:
                    x = np.random.uniform(self.x_min, self.x_max)
                    y = np.random.uniform(self.y_min, self.y_max)
                    vx = np.random.uniform(-self.box_speed, self.box_speed)
                    vy = np.random.uniform(-self.box_speed, self.box_speed)

                # if the box reaches the limits of the gazebo world 
                if x >= self.x_max or x <= self.x_min or y >= self.y_max or y <= self.y_min:
                    # return the box to initial position
                    x = self.box_start_positions[i]['x']
                    y = self.box_start_positions[i]['y']
                    # random in direction
                    vx = np.random.uniform(-self.box_speed, self.box_speed)
                    vy = np.random.uniform(-self.box_speed, self.box_speed)

                # update box data
                self.box_positions[i] = {'x': x, 'y': y, 'vx': vx, 'vy': vy}

                # set model status in gazebo 
                box_state.pose.position.x = x
                box_state.pose.position.y = y
                box_state.pose.position.z = self.box_start_positions[i]['z'] 
                box_state.pose.orientation.x = 0.0
                box_state.pose.orientation.y = 0.0
                box_state.pose.orientation.z = 0.0
                box_state.pose.orientation.w = 1.0

                # Create a request to set the entity state
                request = SetEntityState.Request()
                request.state = box_state

                # Call the service to set the entity state
                future = self.set_state_client.call_async(request)
                future.add_done_callback(self.service_response_callback)

            rate.sleep()

    def service_response_callback(self, future):
        """Callback for handling the service response."""
        try:
            response = future.result()
            if response.success:
                pass
                #self.get_logger().info("Set entity state succeeded")
            else:
                self.get_logger().error("Set entity state failed")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    dynamic_obstacle_node = None  # Initialize the variable outside the try block
    try:
        dynamic_obstacle_node = DynamicObstacleNode()
        rclpy.spin(dynamic_obstacle_node)
    except RuntimeError as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("Node shutdown by user.")
    finally:
        if dynamic_obstacle_node is not None:
            dynamic_obstacle_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

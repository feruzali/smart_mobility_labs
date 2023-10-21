import rclpy
from rclpy.action import ActionClient
from action_fleet_interfaces.action import FleetManagement

def fleet_management_client():
    rclpy.init()

    # Create a node
    node = rclpy.create_node('fleet_management_client')

    # Create an ActionClient
    action_client = ActionClient(node, FleetManagement, 'fleet_management')

    # Wait for the server to be available
    while not action_client.wait_for_server(timeout_sec=1.0):
        node.get_logger().info('Action server not available, waiting...')

    # Create a FleetManagement goal
    goal = FleetManagement.Goal()
    goal.fleet_size = int(input("Input fleet size: "))

    # Send the goal and wait for result
    future = action_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    
    res = FleetManagement.Result()
    print(res.vehicle_routes)

    # Check if the goal succeeded
    if future.result():
        node.get_logger().info('Fleet management action succeeded')
    else:
        node.get_logger().warning('Fleet management action failed')


    # Clean up
    
    action_client.destroy()
    rclpy.shutdown()

def main(args=None):
    fleet_management_client()

if __name__ == '__main__':
    main()


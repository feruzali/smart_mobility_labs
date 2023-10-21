import rclpy
from rclpy.action import ActionServer
from action_fleet_interfaces.action import FleetManagement

class FleetManagementServer:
    def __init__(self):
        self.node = rclpy.create_node('fleet_management_server')

        # Initialize Action Server
        self.action_server = ActionServer(
            self.node,
            FleetManagement,
            'fleet_management',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.node.get_logger().info('Received fleet management request')

        fleet_size = goal_handle.request.fleet_size
        vehicle_routes = simple_route_generation(fleet_size)

        
        result = FleetManagement.Result()
        result.vehicle_routes = [",".join(vehicle_routes[vehicle]) for vehicle in vehicle_routes]

        

        feedback_msg = FleetManagement.Feedback()

        # Publish feedback to indicate progress
        for i in range(1, fleet_size + 1):
            feedback_msg.completion_percentage = i / fleet_size
            goal_handle.publish_feedback(feedback_msg)

        # Set the result and mark the goal as succeeded
        goal_handle.result = result
        goal_handle.succeed()

        self.node.get_logger().info('Fleet management completed successfully')

def simple_route_generation(fleet_size):
    task_locations = ['Seoul', 'Busan', 'Incheon', 'Daegu', 'Daejeon', 'Gwangju', 'Ulsan', 'Suwon', 'Changwon', 'Seongnam', 'Goyang', 'Anyang', 'Yongin', 'Jeonju', 'Cheongju', 'Chuncheon', 'Gangneung', 'Jeju', 'Gimpo', 'Pohang', 'Ansan', 'Wonju', 'Guri', 'Gimhae', 'Yangju', 'Bucheon', 'Asan', 'Gunsan', 'Gyeongju', 'Iksan', 'Mokpo', 'Suncheon', 'Yeosu', 'Jecheon', 'Hwaseong', 'Gwangmyeong', 'Pyeongtaek', 'Uijeongbu', 'Cheonan', 'Andong', 'Gimcheon', 'Gyeongsan', 'Pocheon', 'Gwacheon', 'Naju', 'Gimje', 'Miryang', 'Hwado']

    routes = {}
    num_tasks = len(task_locations)

    for i in range(fleet_size):
        routes[f'Vehicle_{i+1}'] = []

    for i, location in enumerate(task_locations):
        vehicle = f'Vehicle_{i % fleet_size + 1}'
        routes[vehicle].append(location)

    return routes

def main(args=None):
    try:
        rclpy.init(args=args)
        server = FleetManagementServer()
        rclpy.spin(server.node)
        server.node.destroy_node()
        rclpy.shutdown()
    except AssertionError:
            print()

if __name__ == '__main__':
    main()


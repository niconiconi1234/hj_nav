from flask import Flask, request, jsonify
from waitress import serve
from argparse import ArgumentParser
from rclpy.action import ActionClient
import rclpy
import tf2_ros
from nav2_msgs.action import NavigateToPose
try:
    from .utils import eular_to_quaternion, quaternion_to_eular # for ros2 run
except:
    from utils import eular_to_quaternion, quaternion_to_eular # for python3 nav_http_server.py

# 解析--map-frame,--robot-frame和--navigate-to-pose-action三个参数
parser = ArgumentParser()
parser.add_argument('--map-frame', default='map')
parser.add_argument('--robot-frame', default='base_link')
parser.add_argument('--navigate-to-pose-action', default='/navigate_to_pose')
args = parser.parse_args()

MAP_FRAME = args.map_frame
ROBOT_FRAME = args.robot_frame
NAVIGATE_TO_POSE_ACTION = args.navigate_to_pose_action

class SendNavGoalNode(rclpy.node.Node):
    def __init__(self):
        super().__init__('send_nav_goal_node')
        self.action_client = ActionClient(self, NavigateToPose, NAVIGATE_TO_POSE_ACTION)
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available after waiting')
            exit(1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
    
    def send_nav_goal(self, x, y, theta):
        q = eular_to_quaternion(0,0,theta)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = MAP_FRAME
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]
        
        goal_handle_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, goal_handle_future)
        goal_handle = goal_handle_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
        return True

    def robot_loc(self):        
        transform = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
        x = transform.transform.translation.x
        y = transform.transform.translation.y
        
        q = transform.transform.rotation
        e = quaternion_to_eular(q.x,q.y,q.z,q.w)
        theta = e[2]
        
        return x, y, theta

rclpy.init()
node = SendNavGoalNode()

app = Flask(__name__)

@app.route('/nav', methods=['POST'])
def nav():
    # 控制机器人移动到指定位置
    x = request.json['x']
    y = request.json['y']
    theta = request.json['theta']

    # check if the input is valid
    if x is None or y is None or theta is None:
        return jsonify({'status': 'error', 'message': 'invalid input'})

    success = node.send_nav_goal(x, y, theta)
    if success:
        return jsonify({'status': 'success'})
    else:
        return jsonify({'status': 'error', 'message': 'goal rejected'})


@app.route('/loc', methods=['GET'])
def loc():
    x, y, theta = node.robot_loc()
    return jsonify({'x': x, 'y': y, 'theta': theta})
    

def main():
    serve(app, host='0.0.0.0', port=5000)


if __name__ == '__main__':
   main()

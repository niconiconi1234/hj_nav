import rclpy
from std_srvs.srv import Empty
import argparse
from rclpy.node import Node
from datetime import datetime

# 解析--global-localization-service参数
parser = argparse.ArgumentParser()
parser.add_argument('--global-localization-service', default='/reinitialize_global_localization')
args, unknown = parser.parse_known_args()
GLOBAL_LOCALIZATION_SERVICE = args.global_localization_service


class GlobalLocalizationClient(Node):
    def __init__(self):
        ts = datetime.now().timestamp()
        super().__init__('global_localization_client'+str(ts).replace('.', ''))
        self.client = self.create_client(Empty, GLOBAL_LOCALIZATION_SERVICE)
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info(f'service {GLOBAL_LOCALIZATION_SERVICE} not available, waiting again...')

    def global_localization(self):
        req = Empty.Request()
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    node = GlobalLocalizationClient()
    node.global_localization()
    node.get_logger().info(f'global lpcalization by calling service {GLOBAL_LOCALIZATION_SERVICE} success')


if __name__ == '__main__':
    main()

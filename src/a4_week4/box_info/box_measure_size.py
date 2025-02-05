import rclpy
from rclpy.node import Node
from a4_week4_interface.msg import BoxPoint

class BoxSize(Node):
    def __init__(self):
        super().__init__('box_measure')

        self.box_points_sub = self.create_subscription(BoxPoint, 'box_points', self.box_measure, 10)
        self.box_info_pub_ = self.create_publisher(BoxPoint, 'box_info', 10)

    def box_measure(self, box_points):
        # box_points.points를 이용하여 wide, height, center_x, center_y 계산
        wide, height, center_x, center_y = self.box_info(box_points)

        # 메시지 생성 및 데이터 저장
        msg = BoxPoint()
        msg.points = [wide, height, center_x, center_y]  # 🚀 수정됨

        # 로깅
        self.get_logger().info(f'wide = {wide}, height = {height}, center_x = {center_x}, center_y = {center_y}')

        # ROS2 퍼블리시
        self.box_info_pub_.publish(msg)

    def box_info(self, box_points):
        # 🚀 수정: `data` → `points`
        x1, y1, x2, y2 = box_points.points  

        # 넓이, 높이 계산
        wide = x2 - x1
        height = y2 - y1
        center_x, center_y = int((x1 + x2) / 2), int((y1 + y2) / 2)

        return wide, height, center_x, center_y

def main(args=None):
    rclpy.init(args=args)
    node = BoxSize()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

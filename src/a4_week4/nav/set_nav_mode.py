import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import sys
import math

class WaypointNavigator(Node):
    def __init__(self, mode):
        super().__init__('waypoint_navigator')

        #waypoints yaml 파일
        self.waypoint_file = 'waypoints.yaml'
      
        # NavigateToPose 액션 클라이언트 생성
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # 'statue' 토픽 구독 (String 타입 메시지)
        self.subscription = self.create_subscription(
            String,
            'statue',
            self.statue_callback,
            10
        )

        # 터미널에서 mode 인자(argument) 받기 (default: 'searching')
        self.declare_parameter('mode', 'searching')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value

        # waypoints.yaml에서 웨이포인트 로드
        self.waypoints = self.load_waypoints()
        self.current_waypoint_index = 0  # 현재 진행 중인 waypoint 인덱스

        # 현재 실행 중인 goal 핸들 저장 (cancel을 위해 필요)
        self._goal_handle = None

        # 'wait' 모드라면 대기, 그렇지 않으면 목표 전송
        self.is_waiting = self.mode == 'wait'
        self.get_logger().info(f"Mode: {self.mode}")

        # 'searching' 모드일 경우 즉시 웨이포인트 탐색 시작
        if self.mode == 'searching':
            self.send_goal()

    def load_waypoints(self):
        """ waypoints.yaml 파일에서 웨이포인트 로드 """
        yaml_path = os.path.join(get_package_share_directory('a4_week4'), 'waypoints', self.waypoint_file)
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
        return data['waypoints']

    def euler_to_quaternion(self, yaw):
        """ yaw (라디안) 값을 quaternion으로 변환 """
        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)
        return Quaternion(x=0.0, y=0.0, z=qz, w=qw)

    def send_goal(self):
        """ 현재 웨이포인트를 NavigateToPose 액션으로 전송 """
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints completed. Keep searching from start")
            # 마지막 지점 도착 시 처음으로 이동
            self.current_waypoint_index = 0

        # 현재 이동할 waypoint 가져오기
        waypoint = self.waypoints[self.current_waypoint_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint['x']
        goal_msg.pose.pose.position.y = waypoint['y']
        goal_msg.pose.pose.orientation = self.euler_to_quaternion(waypoint['yaw'])

        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}")
        
        # 서버 대기 및 목표 전송
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, self.goal_feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_feedback_callback(self, feedback_msg): pass
  
    def goal_response_callback(self, future):
        """ 목표가 수락되었는지 확인 """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected.")
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        self._goal_handle = goal_handle  # 현재 실행 중인 goal 핸들 저장
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """ 목표 도착 후 다음 웨이포인트로 이동 """
        self.get_logger().info("Reached waypoint!")
        self.current_waypoint_index += 1  # 다음 웨이포인트로 이동

        # 대기 상태가 아니면 다음 웨이포인트 실행
        if not self.is_waiting:
            self.send_goal()

    def cancel_goal(self):
        """ 현재 실행 중인 목표를 취소 """
        if self._goal_handle is not None:
            self.get_logger().info("Cancelling current goal...")
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info("No active goal to cancel.")

    def cancel_done_callback(self, future):
        """ 목표 취소 완료 후 상태 변경 """
        cancel_response = future.result()
        if cancel_response.accepted:
            self.get_logger().info("Goal successfully cancelled.")
        else:
            self.get_logger().warn("Goal cancellation failed.")

    def statue_callback(self, msg):
        """ statue 메시지 수신 시 동작 처리 """
        self.get_logger().info(f"Received statue: {msg.data}")
        
        if msg.data == "cancel":
            # 'cancel' 메시지를 받으면 이동 중이든 아니든 즉시 취소하고 wait 모드로 전환
            self.get_logger().info("Received 'cancel'. Cancelling goal and switching to wait mode.")
            self.cancel_goal()
            self.mode = 'wait'
            self.is_waiting = True
            return
        
        # wait 모드일 때 statue 메시지가 'searching'이면 이동 시작
        if self.mode == 'wait' and msg.data == "searching":
            self.is_waiting = False
            self.send_goal()
        
        # tracking 모드일 때 statue 메시지가 'missing' 또는 'complete'이면 이동 시작
        elif self.mode == 'tracking' and msg.data in ["missing", "complete"]:
            self.is_waiting = False
            self.send_goal()

def main(args=None):
    """ ROS 2 노드 실행 """
    rclpy.init(args=args)

    #터미널 mode 설정 ex) ros2 run a4_week4 set_nav_mode --ros-args -p mode:=wait
    if len(sys.argv) < 2: mode = None
    else : mode = sys.argv[1]

    node = WaypointNavigator(mode)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    '''
        waypoints.yaml을 로드하여 웨이포인트 리스트 생성
        mode 인자 값을 읽어 초기 동작 결정
        → 즉시 탐색 시작
        → statue 메시지를 "searching"으로 받을 때까지 대기
        → statue 메시지를 "missing" 또는 "complete"으로 받을 때까지 대기
        액션을 사용하여 한 개의 waypoint로 이동
        도착 시 다음 웨이포인트로 이동
        토픽을 구독하여 이동 중지 또는 재개
    '''
    main()

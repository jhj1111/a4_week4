import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys
import termios
import tty
import select

class StatuePublisher(Node):
    def __init__(self):
        super().__init__('statue_publisher')
        self.publisher = self.create_publisher(String, 'statue', 10)
        self.get_logger().info("StatuePublisher 노드가 시작되었습니다.")
        
        # 종료 플래그 추가
        self.exit_flag = threading.Event()
        
        # 키보드 입력을 별도의 스레드로 처리
        self.listener_thread = threading.Thread(target=self.keyboard_listener)
        self.listener_thread.daemon = True
        self.listener_thread.start()

    def keyboard_listener(self):
        # 터미널에서 입력을 받기 위해 설정
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while not self.exit_flag.is_set():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    char = sys.stdin.read(1)
                    self.handle_keypress(char)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def handle_keypress(self, char):
        # 키보드 입력에 따른 statue 발행
        msg = String()

        if char == 's':
            msg.data = 'searching'
        elif char == 'm':
            msg.data = 'missing'
        elif char == 'c':
            msg.data = 'cancel'
        elif char == 'f':
            msg.data = 'complete'
        elif char == 'q':
            self.get_logger().info("종료 키(q)를 눌렀습니다. 노드를 종료합니다.")
            self.exit_flag.set()  # 종료 플래그 설정
            return
        else:
            return  # 다른 키는 무시

        self.publisher.publish(msg)
        self.get_logger().info(f'발행된 메시지: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    node = StatuePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('노드가 키보드 인터럽트로 종료되었습니다.')
    finally:
        node.exit_flag.set()  # 종료 플래그 설정
        node.listener_thread.join()  # 스레드 종료 대기


if __name__ == '__main__':
    main()

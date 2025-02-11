import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from a4_week4_interface.msg import BoxPoint
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2, os
from ament_index_python.packages import get_package_share_directory

class YOLOTrackingPublisher(Node):
    def __init__(self):
        super().__init__('yolo_tracking_publisher')
        #self.weigts = 'sample01_50_best.pt'
        self.weigts = 'sample02_best.pt'

        self.publisher_ = self.create_publisher(Image, 'tracked_image', 10)
        self.box_points_pub = self.create_publisher(BoxPoint, 'box_points', 10)

        self.bridge = CvBridge()
        self.pkg_dir = os.path.join(get_package_share_directory('a4_week4'))
        weight_dir = os.path.join(self.pkg_dir, 'weights', self.weigts)
        self.model = YOLO(weight_dir)
        self.class_name = self.model.names

        self.cap = cv2.VideoCapture(2)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Adjust timer frequency as needed
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture frame from webcam.")
            return
        # Run tracking and get results
        tracker_dir = os.path.join(self.pkg_dir, 'trackers')
        tracker_bytetrack = os.path.join(tracker_dir, 'bytetrack.yaml')
        results = self.model.track(source=frame, show=False, tracker=tracker_bytetrack)
        box_points = BoxPoint()

        # Iterate over results to extract data
        for result in results:
            for detection in result.boxes.data:
                if len(detection) >= 6:
                    #x1, y1, x2, y2, confidence, class_id = detection[:6]
                    x1, y1, x2, y2, track_id, confidence = detection[:6]
                    p = detection[:4]
                     # x1, y1, x2, y2 추출 및 int32 변환
                    p = np.array(detection[:4], dtype=np.int32).tolist()

                    # 메시지에 데이터 저장
                    box_points.points = p

                    class_id = detection[6] if len(detection) > 6 else None
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    # Draw bounding box and center point on the frame
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                    label_text = f'Conf: {confidence:.2f} track_id: {int(track_id)}'
                    if track_id is not None:
                        label_text = f'Class: {self.class_name[int(class_id)]}, ' + label_text
                    cv2.putText(frame, label_text, (int(x1), int(y1) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    

        # Convert the frame to a ROS 2 Image message and publish
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.box_points_pub.publish(box_points)
    def destroy_node(self):
        super().destroy_node()
        self.cap.release()
def main(args=None):
    rclpy.init(args=args)
    node = YOLOTrackingPublisher()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
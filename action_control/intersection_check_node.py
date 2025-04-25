#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Point

class IntersectionCheckNode(Node):
    def __init__(self):
        super().__init__('intersection_check_node')
        self.get_logger().info('교차로 체크 노드 초기화 중...')
        
        # TF 리스너 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.tf_check_timer = self.create_timer(0.05, self.check_intersection_tf)
        
        self.intersection_detected = False
        self.intersection_distance = 0.0
        self.intersection_offset_x = 0.0
        self.intersection_offset_y = 0.0
        
        self.intersection_point_pub = self.create_publisher(Point, 'intersection_point', 10)

    def check_intersection_tf(self):
        try:
            # base_link와 intersection_point 사이의 변환 확인
            if self.tf_buffer.can_transform('base_link', 'intersection_point', rclpy.time.Time()):
                # 변환 정보 가져오기
                transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    'intersection_point',
                    rclpy.time.Time()
                )
                
                # 거리 계산 (base_link 기준)
                self.intersection_offset_x = transform.transform.translation.x
                self.intersection_offset_y = transform.transform.translation.y
                self.intersection_distance = math.sqrt(
                    self.intersection_offset_x**2 + 
                    self.intersection_offset_y**2
                )
                
                if not self.intersection_detected:
                    # self.get_logger().info(
                    #     f'교차로 감지됨! 거리: {self.intersection_distance:.2f}m, '
                    #     f'오프셋: ({self.intersection_offset_x:.2f}, {self.intersection_offset_y:.2f})'
                    # )
                    if self.intersection_distance < 2:
                        self.intersection_point_pub.publish(Point(x=self.intersection_offset_x, y=self.intersection_offset_y))
            else:
                if self.intersection_detected:
                    self.get_logger().info('교차로 TF 프레임이 더 이상 감지되지 않습니다.')
                self.intersection_detected = False
                
        except Exception as e:
            if "lookup_transform" not in str(e):  # 일반적인 TF 없음 오류가 아닌 경우만 로깅
                self.get_logger().warn(f'교차로 TF 확인 중 오류 발생: {str(e)}')
            self.intersection_detected = False
    

def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    try:
        # 노드 생성
        node = IntersectionCheckNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

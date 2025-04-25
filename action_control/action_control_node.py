#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum, auto
import time
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from gole_action_interfaces.action import LineTrace, MoveRobot, RotateRobot
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
class RobotState(Enum):
    IDLE = auto()
    START_LINE_TRACING = auto()
    LINE_TRACING = auto()
    END_LINE_TRACING = auto()
    START_MOVE_ROBOT = auto()
    MOVE_ROBOT = auto()
    END_MOVE_ROBOT = auto()
    START_ROTATE_ROBOT = auto()
    ROTATE_ROBOT = auto()
    END_ROTATE_ROBOT = auto()
    
class RotateDirection(Enum):
    LEFT = 90
    RIGHT = -90
    STRAIGHT = 0
    
class ActionControlNode(Node):
    def __init__(self):
        super().__init__('action_control_node')
        
        self.logger = self.get_logger()
        
        # 콜백 그룹 생성 (재진입 가능)
        self.reentrant_callback_group = ReentrantCallbackGroup()
        self.mutually_exclusive_callback_group = MutuallyExclusiveCallbackGroup()
        
        # 액션 클라이언트 생성
        self.line_trace_client = ActionClient(
            self, LineTrace, 'line_trace', callback_group=self.reentrant_callback_group)
        self.move_robot_client = ActionClient(
            self, MoveRobot, 'move_robot', callback_group=self.reentrant_callback_group)
        self.rotate_robot_client = ActionClient(
            self, RotateRobot, 'rotate_robot', callback_group=self.reentrant_callback_group)
        
        self.intersection_position_sub = self.create_subscription(
            Point,
            'intersection_point',
            self.intersection_position_callback,
            10,
            callback_group=self.mutually_exclusive_callback_group
        )
        
        self.left_line_center_sub = self.create_subscription(
            Bool,
            'left/camera/line_center',
            self.left_line_center_callback,
            10,
            callback_group=self.mutually_exclusive_callback_group
        )
        
        self.right_line_center_sub = self.create_subscription(
            Bool,
            'right/camera/line_center',
            self.right_line_center_callback,
            10,
            callback_group=self.mutually_exclusive_callback_group
        )
        
        # 상태 머신 초기화
        self.current_state = RobotState.IDLE
        
        # 현재 목표 저장 변수
        self.current_goal = None
        self.current_action_type = None
        
        # 액션 서버 연결 대기
        self.wait_for_action_servers()
        
        # 주기적인 상태 체크 타이머
        self.create_timer(0.1, self.state_machine_callback, callback_group=self.reentrant_callback_group)
        
        self.logger.info('액션 컨트롤 노드가 시작되었습니다')
        
        self.intersection_flag = False
        self.left_line_center = False
        self.right_line_center = False
        self.recipy = [
            RotateDirection.LEFT,
            RotateDirection.LEFT,
            RotateDirection.STRAIGHT,
            RotateDirection.RIGHT,
        ]
        self.recipy_index = 0
        self.direction = RotateDirection.STRAIGHT
        self.current_state = RobotState.START_LINE_TRACING
        
    def intersection_position_callback(self, msg):
        self.intersection_position = msg
        # self.logger.info(f'교차로 위치: {self.intersection_position}')
        if self.current_state == RobotState.MOVE_ROBOT:
            return
        if self.current_state == RobotState.LINE_TRACING:
            self.stop_line_tracing()
            self.intersection_flag = True
        
    def state_machine_callback(self):
        if self.current_state == RobotState.IDLE:
            pass
        
        elif self.current_state == RobotState.START_LINE_TRACING:
            self.get_logger().info('라인 트레이싱 시작')
            self.start_line_tracing()
            
        elif self.current_state == RobotState.END_LINE_TRACING:
            self.get_logger().info('라인 트레이싱 완료')
            if self.intersection_flag:
                self.current_state = RobotState.START_MOVE_ROBOT
                
        elif self.current_state == RobotState.START_MOVE_ROBOT:
            self.get_logger().info('이동 시작')
            self.move_robot(self.intersection_position.x, self.intersection_position.y, 0.5)
            
        elif self.current_state == RobotState.END_MOVE_ROBOT:
            self.get_logger().info('이동 완료')
            if self.recipy_index >= len(self.recipy):
                self.get_logger().info('레시피 완료')
                self.current_state = RobotState.IDLE
            else:
                if self.left_line_center and self.right_line_center:
                    self.direction = self.recipy[self.recipy_index]
                    self.recipy_index += 1
                elif self.left_line_center:
                    self.direction = RotateDirection.LEFT
                elif self.right_line_center:
                    self.direction = RotateDirection.RIGHT
                else:
                    self.current_state = RobotState.LINE_TRACING
                    
                self.current_state = RobotState.START_ROTATE_ROBOT
        
        elif self.current_state == RobotState.START_ROTATE_ROBOT:
            self.get_logger().info('회전 시작')
            self.rotate_robot(self.direction, 0.5)
        
        elif self.current_state == RobotState.END_ROTATE_ROBOT:
            self.get_logger().info('회전 완료')
            self.current_state = RobotState.START_LINE_TRACING
            
    def left_line_center_callback(self, msg):   
        self.left_line_center = msg
        # self.logger.info(f'왼쪽 라인 중앙: {self.left_line_center}')
    
    def right_line_center_callback(self, msg):
        self.right_line_center = msg
        # self.logger.info(f'오른쪽 라인 중앙: {self.right_line_center}')
    
    def wait_for_action_servers(self):
        """모든 액션 서버에 연결을 시도합니다."""
        self.logger.info('액션 서버 연결 대기 중...')
        
        line_trace_ready = self.line_trace_client.wait_for_server(timeout_sec=5.0)
        if not line_trace_ready:
            self.logger.warn('LineTrace 액션 서버에 연결할 수 없습니다')
        else:
            self.logger.info('LineTrace 액션 서버에 연결되었습니다')
        
        move_robot_ready = self.move_robot_client.wait_for_server(timeout_sec=5.0)
        if not move_robot_ready:
            self.logger.warn('MoveRobot 액션 서버에 연결할 수 없습니다')
        else:
            self.logger.info('MoveRobot 액션 서버에 연결되었습니다')
        
        rotate_robot_ready = self.rotate_robot_client.wait_for_server(timeout_sec=5.0)
        if not rotate_robot_ready:
            self.logger.warn('RotateRobot 액션 서버에 연결할 수 없습니다')
        else:
            self.logger.info('RotateRobot 액션 서버에 연결되었습니다')
    
    def start_line_tracing(self):
        """라인 트레이싱을 시작합니다."""
        self.current_action_type = 'line_trace'
        self.current_state = RobotState.LINE_TRACING
            
        # 라인 트레이싱 목표 생성
        goal_msg = LineTrace.Goal()
        goal_msg.start = True
        self.current_goal = goal_msg
        
        self.logger.info('라인 트레이싱 시작')
        self._send_line_trace_goal(goal_msg)

    def stop_line_tracing(self):
        """라인 트레이싱을 중지합니다."""
        goal_msg = LineTrace.Goal()
            # 라인 트레이싱 중지 목표 생성
        goal_msg = LineTrace.Goal()
        goal_msg.start = False
        
        self.logger.info('라인 트레이싱 중지')
        self._send_line_trace_goal(goal_msg)
    
    def move_robot(self, target_x, target_y, linear_speed=0.2):
        """로봇을 특정 위치로 이동시킵니다."""
        self.current_action_type = 'move'
        self.current_state = RobotState.MOVE_ROBOT
        # 이동 목표 생성
        goal_msg = MoveRobot.Goal()
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y
        goal_msg.linear_speed = linear_speed
        self.current_goal = goal_msg
        
        self.logger.info(f'이동 시작: ({target_x}, {target_y})')
        self._send_move_goal(goal_msg)
    
    
    def rotate_robot(self, target_angle, angular_speed=0.5):
        """로봇을 특정 각도로 회전시킵니다."""
        self.current_action_type = 'rotate'
        self.current_state = RobotState.ROTATE_ROBOT
        # 회전 목표 생성
        goal_msg = RotateRobot.Goal()
        goal_msg.target_angle = float(target_angle)
        goal_msg.angular_speed = float(angular_speed)
        self.current_goal = goal_msg
        
        self.logger.info(f'회전 시작: {target_angle}도')
        self._send_rotate_goal(goal_msg)
    
    
    def _send_line_trace_goal(self, goal_msg):
        """LineTrace 액션 목표를 전송합니다."""
        self.line_trace_client.wait_for_server()
        
        send_goal_future = self.line_trace_client.send_goal_async(
            goal_msg,
            feedback_callback=self._line_trace_feedback_callback
        )
        
        send_goal_future.add_done_callback(self._line_trace_goal_response_callback)
    
    def _send_move_goal(self, goal_msg):
        """MoveRobot 액션 목표를 전송합니다."""
        self.move_robot_client.wait_for_server()
        
        send_goal_future = self.move_robot_client.send_goal_async(
            goal_msg,
            feedback_callback=self._move_robot_feedback_callback
        )
        
        send_goal_future.add_done_callback(self._move_robot_goal_response_callback)
    
    def _send_rotate_goal(self, goal_msg):
        """RotateRobot 액션 목표를 전송합니다."""
        self.rotate_robot_client.wait_for_server()
        
        send_goal_future = self.rotate_robot_client.send_goal_async(
            goal_msg,
            feedback_callback=self._rotate_robot_feedback_callback
        )
        
        send_goal_future.add_done_callback(self._rotate_robot_goal_response_callback)
    
    # LineTrace 액션 콜백 함수들
    def _line_trace_feedback_callback(self, feedback_msg):
        """LineTrace 액션의 피드백을 처리합니다."""
        feedback = feedback_msg.feedback
        self.logger.debug(
            f'라인 트레이싱 피드백: y_error={feedback.current_y_error:.3f}, '
            f'angular={feedback.current_angular_speed:.3f}, '
            f'linear={feedback.current_linear_speed:.3f}'
        )
    
    def _line_trace_goal_response_callback(self, future):
        """LineTrace 액션 목표 응답을 처리합니다."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.logger.error('라인 트레이싱 목표가 거부되었습니다')
            self.current_state = RobotState.END_LINE_TRACING
            self.current_action_type = None
            self.current_goal = None
            return
        
        self.logger.info('라인 트레이싱 목표가 수락되었습니다')
        
        # 결과 받기
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._line_trace_result_callback)
    
    def _line_trace_result_callback(self, future):
        """LineTrace 액션 결과를 처리합니다."""
        result = future.result().result
        
        if result.success:
            self.logger.info(f'라인 트레이싱 완료: {result.message}')
        else:
            self.logger.error(f'라인 트레이싱 실패: {result.message}')
        
        self.current_state = RobotState.END_LINE_TRACING
        self.current_action_type = None
        self.current_goal = None
    
    # MoveRobot 액션 콜백 함수들
    def _move_robot_feedback_callback(self, feedback_msg):
        """MoveRobot 액션의 피드백을 처리합니다."""
        feedback = feedback_msg.feedback
        self.logger.debug(
            f'이동 피드백: 위치=({feedback.current_x:.3f}, {feedback.current_y:.3f}), '
            f'남은 거리={feedback.distance_remaining:.3f}, '
            f'linear={feedback.current_linear_speed:.3f}, '
            f'angular={feedback.current_angular_speed:.3f}'
        )
    
    def _move_robot_goal_response_callback(self, future):
        """MoveRobot 액션 목표 응답을 처리합니다."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.logger.error('이동 목표가 거부되었습니다')
            self.current_state = RobotState.END_MOVE_ROBOT
            self.current_action_type = None
            self.current_goal = None
            return
        
        self.logger.info('이동 목표가 수락되었습니다')
        
        # 결과 받기
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._move_robot_result_callback)
    
    def _move_robot_result_callback(self, future):
        """MoveRobot 액션 결과를 처리합니다."""
        result = future.result().result
        
        if result.success:
            self.logger.info(
                f'이동 완료: 최종 위치=({result.final_x:.3f}, {result.final_y:.3f}), '
                f'메시지: {result.message}'
            )
        else:
            self.logger.error(f'이동 실패: {result.message}')
        
        # 이동 완료 후 IDLE 상태로 전환
        self.current_state = RobotState.END_MOVE_ROBOT
        self.current_action_type = None
        self.current_goal = None
    
    # RotateRobot 액션 콜백 함수들
    def _rotate_robot_feedback_callback(self, feedback_msg):
        """RotateRobot 액션의 피드백을 처리합니다."""
        feedback = feedback_msg.feedback
        self.logger.debug(
            f'회전 피드백: 현재 각도={feedback.current_angle:.3f}, '
            f'남은 각도={feedback.remaining_angle:.3f}, '
            f'angular={feedback.current_angular_speed:.3f}, '
            f'linear={feedback.current_linear_speed:.3f}'
        )
    
    def _rotate_robot_goal_response_callback(self, future):
        """RotateRobot 액션 목표 응답을 처리합니다."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.logger.error('회전 목표가 거부되었습니다')
            self.current_state = RobotState.END_ROTATE_ROBOT
            self.current_action_type = None
            self.current_goal = None
            return
        
        self.logger.info('회전 목표가 수락되었습니다')
        
        # 결과 받기
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._rotate_robot_result_callback)
    
    def _rotate_robot_result_callback(self, future):
        """RotateRobot 액션 결과를 처리합니다."""
        result = future.result().result
        
        if result.success:
            self.logger.info(
                f'회전 완료: 최종 각도={result.final_angle:.3f}, '
                f'메시지: {result.message}'
            )
        else:
            self.logger.error(f'회전 실패: {result.message}')
        
        # 회전 완료 후 IDLE 상태로 전환
        self.current_state = RobotState.END_ROTATE_ROBOT
        self.current_action_type = None
        self.current_goal = None


def main(args=None):
    rclpy.init(args=args)
    
    action_control_node = ActionControlNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(action_control_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

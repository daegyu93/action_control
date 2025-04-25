#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from enum import Enum, auto
import threading
import time
import math

# 액션 인터페이스 임포트
from gole_action_interfaces.action import LineTrace, MoveRobot, RotateRobot

# 모션 미디에이터 임포트
from action_control.motion_mediator import MotionMediator

class RobotState(Enum):
    """로봇의 상태를 표현하는 열거형"""
    IDLE = auto()               # 대기 상태
    LINE_TRACING = auto()       # 라인 트레이싱 중
    MOVING = auto()             # 이동 중
    ROTATING = auto()           # 회전 중
    STOPPING = auto()           # 정지 중
    ERROR = auto()              # 오류 상태

class ActionControllerNode(Node):
    """
    액션 컨트롤러 노드 - 라인 트레이스, 로봇 이동, 로봇 회전 액션을 통합 관리
    """
    def __init__(self):
        super().__init__('action_controller_node')
        self.get_logger().info('액션 컨트롤러 노드 초기화 중...')
        
        # 콜백 그룹 생성
        self.action_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        
        # 모션 미디에이터 초기화
        self.motion_mediator = MotionMediator(self)
        
        # 상태 변수
        self.state = RobotState.IDLE
        self.state_lock = threading.Lock()
        
        # 액션 클라이언트 초기화
        self.line_trace_client = ActionClient(
            self,
            LineTrace,
            'line_trace',
            callback_group=self.action_cb_group
        )
        
        self.move_robot_client = ActionClient(
            self,
            MoveRobot,
            'move_robot',
            callback_group=self.action_cb_group
        )
        
        self.rotate_robot_client = ActionClient(
            self,
            RotateRobot,
            'rotate_robot',
            callback_group=self.action_cb_group
        )
        
        # 현재 실행 중인 목표
        self.current_goal_handle = None
        self.goal_lock = threading.Lock()
        
        # 사용자 명령 처리를 위한 타이머
        self.command_timer = self.create_timer(
            0.5,  # 2Hz
            self.process_user_commands,
            callback_group=self.timer_cb_group
        )
        
        # 클라이언트 서버 연결 확인
        self.wait_for_action_servers()
        
        self.get_logger().info('액션 컨트롤러 노드가 준비되었습니다.')
    
    def wait_for_action_servers(self):
        """모든 액션 서버에 대한 연결 대기"""
        self.get_logger().info('액션 서버 연결 대기 중...')
        
        servers = {
            '라인 트레이스': self.line_trace_client,
            '로봇 이동': self.move_robot_client,
            '로봇 회전': self.rotate_robot_client
        }
        
        for name, client in servers.items():
            connected = client.wait_for_server(timeout_sec=5.0)
            if connected:
                self.get_logger().info(f'{name} 서버에 연결되었습니다.')
            else:
                self.get_logger().warn(f'{name} 서버에 연결할 수 없습니다!')
    
    def set_state(self, new_state):
        """스레드 안전하게 상태 변경"""
        with self.state_lock:
            old_state = self.state
            self.state = new_state
            
        self.get_logger().info(f'상태 변경: {old_state.name} -> {new_state.name}')
        
        # 상태가 변경될 때 필요한 액션 수행
        if new_state == RobotState.STOPPING:
            self.motion_mediator.stop()
            self.motion_mediator.set_active_controller('stop')
    
    def process_user_commands(self):
        """사용자 명령 처리 (예시 - 이 부분은 실제 사용에 맞게 수정 필요)"""
        # 참고: 실제 구현에서는 이 메서드를 다른 방식(토픽 구독, 서비스 등)으로 대체할 수 있음
        pass
    
    def cancel_current_goal(self):
        """현재 실행 중인 goal 취소"""
        with self.goal_lock:
            if self.current_goal_handle:
                self.get_logger().info('현재 실행 중인 goal 취소 중...')
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
    
    #
    # 라인 트레이스 액션 메서드
    #
    def start_line_tracing(self):
        """라인 트레이싱 시작"""
        # 다른 액션이 실행 중이면 취소
        self.cancel_current_goal()
        
        # 컨트롤러 설정
        self.motion_mediator.set_active_controller('line_trace')
        
        # 상태 변경
        self.set_state(RobotState.LINE_TRACING)
        
        # 목표 생성
        goal_msg = LineTrace.Goal()
        goal_msg.start = True
        
        # 액션 실행
        self.get_logger().info('라인 트레이싱 시작 중...')
        self.line_trace_client.wait_for_server()
        
        send_goal_future = self.line_trace_client.send_goal_async(
            goal_msg,
            feedback_callback=self.line_trace_feedback_callback
        )
        send_goal_future.add_done_callback(self.line_trace_goal_response_callback)
    
    def stop_line_tracing(self):
        """라인 트레이싱 중지"""
        if self.state == RobotState.LINE_TRACING:
            goal_msg = LineTrace.Goal()
            goal_msg.start = False
            
            self.get_logger().info('라인 트레이싱 중지 중...')
            self.line_trace_client.wait_for_server()
            
            send_goal_future = self.line_trace_client.send_goal_async(
                goal_msg,
                feedback_callback=self.line_trace_feedback_callback
            )
            send_goal_future.add_done_callback(self.line_trace_goal_response_callback)
            
            self.set_state(RobotState.STOPPING)
    
    def line_trace_goal_response_callback(self, future):
        """라인 트레이스 목표 응답 콜백"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('라인 트레이스 goal이 거부되었습니다!')
            self.set_state(RobotState.ERROR)
            return
        
        with self.goal_lock:
            self.current_goal_handle = goal_handle
        
        # 결과 요청
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.line_trace_result_callback)
    
    def line_trace_feedback_callback(self, feedback_msg):
        """라인 트레이스 피드백 콜백"""
        feedback = feedback_msg.feedback
        # 필요한 경우 피드백 처리
        # self.get_logger().debug(f'라인 트레이스 피드백: 오차={feedback.current_y_error:.3f}')
    
    def line_trace_result_callback(self, future):
        """라인 트레이스 결과 콜백"""
        result = future.result().result
        
        if self.state == RobotState.LINE_TRACING:
            if result.success:
                self.get_logger().info(f'라인 트레이스 성공: {result.message}')
                self.set_state(RobotState.IDLE)
            else:
                self.get_logger().warn(f'라인 트레이스 실패: {result.message}')
                self.set_state(RobotState.ERROR)
        
        with self.goal_lock:
            self.current_goal_handle = None
    
    #
    # 로봇 이동 액션 메서드
    #
    def move_robot(self, target_x, target_y, speed=0.2):
        """로봇을 지정된 상대 위치로 이동"""
        # 다른 액션이 실행 중이면 취소
        self.cancel_current_goal()
        
        # 컨트롤러 설정
        self.motion_mediator.set_active_controller('move_robot')
        
        # 상태 변경
        self.set_state(RobotState.MOVING)
        
        # 목표 생성
        goal_msg = MoveRobot.Goal()
        goal_msg.target_x = float(target_x)
        goal_msg.target_y = float(target_y)
        goal_msg.linear_speed = float(speed)
        
        # 액션 실행
        self.get_logger().info(f'로봇 이동 시작: 목표=({target_x:.2f}, {target_y:.2f}), 속도={speed:.2f}')
        self.move_robot_client.wait_for_server()
        
        send_goal_future = self.move_robot_client.send_goal_async(
            goal_msg,
            feedback_callback=self.move_robot_feedback_callback
        )
        send_goal_future.add_done_callback(self.move_robot_goal_response_callback)
    
    def move_robot_goal_response_callback(self, future):
        """로봇 이동 목표 응답 콜백"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('로봇 이동 goal이 거부되었습니다!')
            self.set_state(RobotState.ERROR)
            return
        
        with self.goal_lock:
            self.current_goal_handle = goal_handle
        
        # 결과 요청
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.move_robot_result_callback)
    
    def move_robot_feedback_callback(self, feedback_msg):
        """로봇 이동 피드백 콜백"""
        feedback = feedback_msg.feedback
        # 필요한 경우 피드백 처리
        # self.get_logger().debug(
        #     f'로봇 이동 피드백: 위치=({feedback.current_x:.2f}, {feedback.current_y:.2f}), '
        #     f'남은 거리={feedback.distance_remaining:.2f}'
        # )
    
    def move_robot_result_callback(self, future):
        """로봇 이동 결과 콜백"""
        result = future.result().result
        
        if self.state == RobotState.MOVING:
            if result.success:
                self.get_logger().info(
                    f'로봇 이동 성공: 최종 위치=({result.final_x:.2f}, {result.final_y:.2f}), {result.message}'
                )
                self.set_state(RobotState.IDLE)
            else:
                self.get_logger().warn(f'로봇 이동 실패: {result.message}')
                self.set_state(RobotState.ERROR)
        
        with self.goal_lock:
            self.current_goal_handle = None
    
    #
    # 로봇 회전 액션 메서드
    #
    def rotate_robot(self, angle, speed=0.5):
        """로봇을 지정된 각도로 회전"""
        # 다른 액션이 실행 중이면 취소
        self.cancel_current_goal()
        
        # 컨트롤러 설정
        self.motion_mediator.set_active_controller('rotate_robot')
        
        # 상태 변경
        self.set_state(RobotState.ROTATING)
        
        # 목표 생성
        goal_msg = RotateRobot.Goal()
        goal_msg.target_angle = float(angle)
        goal_msg.angular_speed = float(speed)
        
        # 액션 실행
        self.get_logger().info(f'로봇 회전 시작: 각도={angle:.2f}, 속도={speed:.2f}')
        self.rotate_robot_client.wait_for_server()
        
        send_goal_future = self.rotate_robot_client.send_goal_async(
            goal_msg,
            feedback_callback=self.rotate_robot_feedback_callback
        )
        send_goal_future.add_done_callback(self.rotate_robot_goal_response_callback)
    
    def rotate_robot_goal_response_callback(self, future):
        """로봇 회전 목표 응답 콜백"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('로봇 회전 goal이 거부되었습니다!')
            self.set_state(RobotState.ERROR)
            return
        
        with self.goal_lock:
            self.current_goal_handle = goal_handle
        
        # 결과 요청
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.rotate_robot_result_callback)
    
    def rotate_robot_feedback_callback(self, feedback_msg):
        """로봇 회전 피드백 콜백"""
        feedback = feedback_msg.feedback
        # 필요한 경우 피드백 처리
        # self.get_logger().debug(
        #     f'로봇 회전 피드백: 현재 각도={feedback.current_angle:.2f}, '
        #     f'남은 각도={feedback.remaining_angle:.2f}'
        # )
    
    def rotate_robot_result_callback(self, future):
        """로봇 회전 결과 콜백"""
        result = future.result().result
        
        if self.state == RobotState.ROTATING:
            if result.success:
                self.get_logger().info(f'로봇 회전 성공: 최종 각도={result.final_angle:.2f}, {result.message}')
                self.set_state(RobotState.IDLE)
            else:
                self.get_logger().warn(f'로봇 회전 실패: {result.message}')
                self.set_state(RobotState.ERROR)
        
        with self.goal_lock:
            self.current_goal_handle = None
    
def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    try:
        # 노드 생성
        action_controller = ActionControllerNode()
        
        # MultiThreadedExecutor 사용 (여러 콜백을 동시에 처리하기 위함)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(action_controller)
        
        try:
            # 예시: 복합 작업 실행
            # 참고: 실제 사용 시에는 외부 명령(예: 서비스 또는 토픽)으로 트리거해야 함
            # action_controller.execute_task_sequence()
            
            # 실행
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            action_controller.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from enum import Enum, auto
from gole_action_interfaces.action import LineTrace, MoveRobot, RotateRobot
from geometry_msgs.msg import Point, Twist
import asyncio
from collections import deque

class IntersectionAction(Enum):
    """교차로에서 수행할 액션 유형"""
    GO_STRAIGHT = auto()  # 직진
    TURN_LEFT = auto()    # 좌회전
    TURN_RIGHT = auto()   # 우회전
    STOP = auto()         # 정지

class ActionControllerState(Enum):
    """액션 컨트롤러의 상태를 나타내는 열거형"""
    IDLE = auto()           # 대기 상태
    LINE_TRACING = auto()   # 라인 트레이싱 중
    MOVING = auto()         # 로봇 이동 중
    ROTATING = auto()       # 로봇 회전 중

class ActionControllerNode(Node):
    def __init__(self):
        super().__init__('action_controller_node')
        self.get_logger().info('액션 컨트롤러 노드 초기화 중...')
        
        # 상태 및 액션 관련 변수
        self.current_state = ActionControllerState.IDLE
        self.current_action = None
        self.is_action_active = False
        
        # 회전 각도 배열 및 인덱스
        self.rotation_angles = deque([90.0, 90.0, 0])  # 회전 각도 목록 (임의 예시)
        self.next_target_point = None  # 다음 이동 목표점
        self.mission_complete = False  # 미션 완료 여부
        
        # 콜백 그룹 생성
        self.action_cb_group = ReentrantCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        
        # cmd_vel 퍼블리셔 생성
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'diff_drive_controller/cmd_vel_unstamped',
            10
        )
        
        # 교차로 포인트 구독
        self.intersection_point_subscriber = self.create_subscription(
            Point,
            'intersection_point',
            self.intersection_point_callback,
            10
        )
        
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

        # 상태 모니터링 타이머
        self.create_timer(0.1, self.state_monitor_callback, callback_group=self.timer_cb_group)

    async def state_monitor_callback(self):
        """상태를 모니터링하고 필요한 전환을 처리하는 콜백"""
        if self.mission_complete and not self.is_action_active:
            # 미션 완료 및 액션이 없을 때 정지
            self.publish_zero_velocity()
            self.get_logger().info('미션 완료: 모든 회전 각도 처리 완료')

    def publish_zero_velocity(self):
        """0 속도를 발행하여 로봇 정지"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)

    def publish_velocity(self, linear_x, angular_z):
        """속도 명령 발행"""
        twist_msg = Twist()
        twist_msg.linear.x = linear_x
        twist_msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist_msg)
        
    async def execute_action(self, action_type, params=None):
        """액션 실행 통합 함수"""
        # 이전 액션이 실행 중이면 취소하지만 정지는 하지 않음
        if self.is_action_active:
            if self.current_state == ActionControllerState.LINE_TRACING and action_type != ActionControllerState.LINE_TRACING:
                await self.cancel_current_action(stop_robot=False)
            elif self.current_state != ActionControllerState.LINE_TRACING:
                self.get_logger().warn('다른 액션이 실행 중입니다')
                return False
        
        # 액션 타입에 따라 다른 목표 메시지 생성
        goal_msg = None
        client = None
        
        if action_type == ActionControllerState.LINE_TRACING:
            goal_msg = LineTrace.Goal()
            client = self.line_trace_client
            self.current_state = ActionControllerState.LINE_TRACING
            
        elif action_type == ActionControllerState.MOVING:
            goal_msg = MoveRobot.Goal()
            if params:
                goal_msg.target_x = params[0]
                goal_msg.target_y = params[1]
            client = self.move_robot_client
            self.current_state = ActionControllerState.MOVING
            
        elif action_type == ActionControllerState.ROTATING:
            goal_msg = RotateRobot.Goal()
            if params:
                goal_msg.target_angle = params
            client = self.rotate_robot_client
            self.current_state = ActionControllerState.ROTATING
            
        # 액션 요청 전송
        send_goal_future = await client.send_goal_async(goal_msg, feedback_callback=self.action_feedback_callback)
        self.current_action = send_goal_future
        
        # 액션 완료 콜백 설정
        send_goal_future.add_done_callback(self.action_completion_callback)
        self.is_action_active = True
        
        return True

    def action_feedback_callback(self, feedback_msg):
        """액션 피드백을 받아 cmd_vel로 발행"""
        feedback = feedback_msg.feedback
        linear_x = 0.0
        angular_z = 0.0
        
        # 액션 타입에 따라 피드백 처리
        if self.current_state == ActionControllerState.LINE_TRACING:
            linear_x = feedback.linear_velocity
            angular_z = feedback.angular_velocity
        elif self.current_state == ActionControllerState.MOVING:
            linear_x = feedback.linear_velocity
            angular_z = feedback.angular_velocity
        elif self.current_state == ActionControllerState.ROTATING:
            angular_z = feedback.angular_velocity
            
        # 속도 발행
        self.publish_velocity(linear_x, angular_z)

    def action_completion_callback(self, future):
        """액션 완료 시 호출되는 콜백"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('액션이 거부되었습니다')
            return

        # 결과 받기를 위한 future 생성
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.action_result_callback)

    def action_result_callback(self, future):
        """액션 결과 처리 콜백"""
        try:
            result = future.result().result
            self.get_logger().info(f'액션 완료됨: {self.current_state}')
            
            # 액션 완료 후 다음 동작 결정
            if self.current_state == ActionControllerState.MOVING:
                # 이동 완료 후 회전
                if self.rotation_angles:
                    next_angle = self.rotation_angles.popleft()
                    self.get_logger().info(f'다음 회전 각도: {next_angle}')
                    asyncio.create_task(self.execute_action(ActionControllerState.ROTATING, next_angle))
                else:
                    self.get_logger().info('모든 회전 각도 소진됨, 미션 완료')
                    self.mission_complete = True
                    self.is_action_active = False
                    self.current_state = ActionControllerState.IDLE
                    self.publish_zero_velocity()
            
            elif self.current_state == ActionControllerState.ROTATING:
                # 회전 완료 후 라인 트레이스
                self.get_logger().info('회전 완료: 라인 트레이스로 전환')
                asyncio.create_task(self.execute_action(ActionControllerState.LINE_TRACING))
            
            elif self.current_state == ActionControllerState.LINE_TRACING:
                # 라인 트레이스는 일반적으로 교차로 감지로 종료됨
                # 여기서는 별도 처리 없음
                self.is_action_active = False
                self.current_state = ActionControllerState.IDLE
            
        except Exception as e:
            self.get_logger().error(f'액션 결과 처리 중 오류 발생: {str(e)}')

    async def cancel_current_action(self, stop_robot=True):
        """현재 실행 중인 액션 취소"""
        if self.current_action and self.is_action_active:
            await self.current_action.cancel_goal_async()
            self.is_action_active = False
            self.current_action = None
            if stop_robot:
                self.publish_zero_velocity()
            self.get_logger().info('현재 액션이 취소되었습니다')

    def intersection_point_callback(self, msg: Point):
        """교차로 지점 감지 시 콜백"""
        if self.current_state == ActionControllerState.LINE_TRACING:
            # 이동 목표점 저장
            self.next_target_point = (msg.x, msg.y)
            # 이동 액션 시작
            self.get_logger().info(f'교차로 감지: 좌표 ({msg.x}, {msg.y})로 이동')
            asyncio.create_task(self.execute_action(ActionControllerState.MOVING, (msg.x, msg.y)))
    
def main(args=None):
    rclpy.init(args=args)
    action_controller_node = ActionControllerNode()
    rclpy.spin(action_controller_node)
    action_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from enum import Enum, auto
import threading
import time
import math
import numpy as np
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

# ActionControllerNode에서 필요한 기능 임포트
from action_control.action_controller_node import ActionControllerNode, RobotState

class IntersectionAction(Enum):
    """교차로에서 수행할 액션 유형"""
    GO_STRAIGHT = auto()  # 직진
    TURN_LEFT = auto()    # 좌회전
    TURN_RIGHT = auto()   # 우회전
    STOP = auto()         # 정지

class IntersectionState(Enum):
    """교차로 처리 상태"""
    IDLE = auto()                  # 대기 상태
    DETECTING = auto()             # 교차로 감지 중
    APPROACHING = auto()           # 교차로로 접근 중
    ALIGNING = auto()              # 교차로에 정렬 중
    EXECUTING_ACTION = auto()      # 교차로 액션 실행 중
    RESUMING_LINE_TRACE = auto()   # 라인 트레이싱 재개 중
    COMPLETED = auto()             # 교차로 처리 완료

class IntersectionHandler(Node):
    """
    교차로 처리 노드
    
    역할:
    1. intersection_point TF 프레임 모니터링
    2. 교차로 감지 시 로봇을 정확하게 교차로 위치로 이동
    3. 레시피에 따라 적절한 회전 또는 액션 수행
    4. 액션 완료 후 라인 트레이싱 재개
    """
    def __init__(self):
        super().__init__('intersection_handler')
        self.get_logger().info('교차로 처리 노드 초기화 중...')
        
        # 액션 컨트롤러 노드 초기화
        self.controller = ActionControllerNode()
        
        # TF 리스너 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 상태 변수
        self.state = IntersectionState.IDLE
        self.state_lock = threading.Lock()
        
        # 교차로 감지 및 처리를 위한 변수들
        self.intersection_detected = False
        self.intersection_distance = 0.0
        self.intersection_offset_x = 0.0
        self.intersection_offset_y = 0.0
        
        # 레시피 관련 변수
        self.current_recipe_index = 0
        self.intersection_recipes = [
            IntersectionAction.TURN_LEFT,
            IntersectionAction.TURN_RIGHT,
            IntersectionAction.GO_STRAIGHT
        ]  # 기본 레시피 예시
        
        # 레시피 구독자 (동적 레시피 업데이트를 위함)
        self.recipe_subscriber = self.create_subscription(
            String,
            'intersection_recipe',
            self.recipe_callback,
            10
        )
        
        # 상태 업데이트 타이머
        self.update_timer = self.create_timer(0.1, self.update_state)
        
        # TF 감지 타이머
        self.tf_check_timer = self.create_timer(0.05, self.check_intersection_tf)
        
        self.get_logger().info('교차로 처리 노드가 준비되었습니다.')
    
    def recipe_callback(self, msg):
        """레시피 메시지 콜백 - 새로운 교차로 처리 레시피 업데이트"""
        try:
            recipe_str = msg.data.strip().upper()
            new_recipe = []
            
            for char in recipe_str:
                if char == 'S':
                    new_recipe.append(IntersectionAction.GO_STRAIGHT)
                elif char == 'L':
                    new_recipe.append(IntersectionAction.TURN_LEFT)
                elif char == 'R':
                    new_recipe.append(IntersectionAction.TURN_RIGHT)
                elif char == 'X':
                    new_recipe.append(IntersectionAction.STOP)
            
            if new_recipe:
                self.intersection_recipes = new_recipe
                self.current_recipe_index = 0
                self.get_logger().info(f'새로운 교차로 레시피 설정됨: {recipe_str}')
            else:
                self.get_logger().warn('잘못된 레시피 포맷입니다. 예: "SLRS" (직진,좌회전,우회전,정지)')
                
        except Exception as e:
            self.get_logger().error(f'레시피 처리 중 오류 발생: {str(e)}')
    
    def set_state(self, new_state):
        """스레드 안전하게 상태 변경"""
        with self.state_lock:
            old_state = self.state
            self.state = new_state
            
        self.get_logger().info(f'교차로 처리 상태 변경: {old_state.name} -> {new_state.name}')
    
    def check_intersection_tf(self):
        """intersection_point TF 프레임 확인"""
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
                    self.get_logger().info(
                        f'교차로 감지됨! 거리: {self.intersection_distance:.2f}m, '
                        f'오프셋: ({self.intersection_offset_x:.2f}, {self.intersection_offset_y:.2f})'
                    )
                
                self.intersection_detected = True
                
                # 상태가 IDLE이면 DETECTING으로 변경
                if self.state == IntersectionState.IDLE:
                    self.set_state(IntersectionState.DETECTING)
                
            else:
                if self.intersection_detected:
                    self.get_logger().info('교차로 TF 프레임이 더 이상 감지되지 않습니다.')
                
                self.intersection_detected = False
                
        except Exception as e:
            if "lookup_transform" not in str(e):  # 일반적인 TF 없음 오류가 아닌 경우만 로깅
                self.get_logger().warn(f'교차로 TF 확인 중 오류 발생: {str(e)}')
            self.intersection_detected = False
    
    def get_current_action(self):
        """현재 교차로에서 수행할 액션 반환"""
        if not self.intersection_recipes:
            return IntersectionAction.GO_STRAIGHT  # 기본값
        
        # 레시피 인덱스가 범위를 벗어나면 마지막 액션 반복
        if self.current_recipe_index >= len(self.intersection_recipes):
            return self.intersection_recipes[-1]
        
        return self.intersection_recipes[self.current_recipe_index]
    
    def next_action(self):
        """다음 레시피 액션으로 이동"""
        self.current_recipe_index += 1
        if self.current_recipe_index >= len(self.intersection_recipes):
            self.get_logger().info('모든 교차로 레시피를 처리했습니다. 마지막 액션을 반복합니다.')
    
    def update_state(self):
        """상태 머신 업데이트"""
        # 현재 상태 가져오기
        with self.state_lock:
            current_state = self.state
        
        # 로봇 상태 가져오기
        robot_state = self.controller.state
        
        # 상태에 따른 처리
        if current_state == IntersectionState.IDLE:
            # 라인 트레이싱 중이 아니면 시작
            if robot_state != RobotState.LINE_TRACING:
                self.controller.start_line_tracing()
            
        elif current_state == IntersectionState.DETECTING:
            # 교차로가 감지되면 거리에 따라 접근 상태로 전환
            if self.intersection_detected:
                # 충분히 가까워지면 라인 트레이싱 중지 후 교차로로 이동
                if self.intersection_distance < 0.8:  # 예: 80cm 이내
                    self.controller.stop_line_tracing()
                    self.set_state(IntersectionState.APPROACHING)
            else:
                # 교차로가 더 이상 감지되지 않으면 IDLE로 돌아감
                self.set_state(IntersectionState.IDLE)
        
        elif current_state == IntersectionState.APPROACHING:
            # 라인 트레이싱이 중지될 때까지 대기
            if robot_state == RobotState.IDLE:
                # 교차로 위치로 정확히 이동
                self.controller.move_robot(
                    self.intersection_offset_x,
                    self.intersection_offset_y,
                    0.1  # 느린 속도로 이동
                )
                self.set_state(IntersectionState.ALIGNING)
            
        elif current_state == IntersectionState.ALIGNING:
            # 교차로 위치로 이동 완료되면 액션 실행
            if robot_state == RobotState.IDLE:
                # 현재 레시피에 따른 액션 실행
                action = self.get_current_action()
                
                self.get_logger().info(f'교차로 액션 실행: {action.name}')
                
                if action == IntersectionAction.GO_STRAIGHT:
                    # 직진은 별도 액션없이 라인 트레이싱 재개
                    self.set_state(IntersectionState.RESUMING_LINE_TRACE)
                
                elif action == IntersectionAction.TURN_LEFT:
                    # 좌회전 (90도)
                    self.controller.rotate_robot(90.0, 0.4)
                    self.set_state(IntersectionState.EXECUTING_ACTION)
                
                elif action == IntersectionAction.TURN_RIGHT:
                    # 우회전 (-90도)
                    self.controller.rotate_robot(-90.0, 0.4)
                    self.set_state(IntersectionState.EXECUTING_ACTION)
                
                elif action == IntersectionAction.STOP:
                    # 정지 (1초 대기 후 다음 상태로)
                    self.get_logger().info('교차로에서 잠시 정지합니다.')
                    # 실제 구현에서는 타이머 콜백 등으로 대기 시간 구현
                    time.sleep(1.0)
                    self.set_state(IntersectionState.RESUMING_LINE_TRACE)
            
        elif current_state == IntersectionState.EXECUTING_ACTION:
            # 액션 완료되면 라인 트레이싱 재개
            if robot_state == RobotState.IDLE:
                self.set_state(IntersectionState.RESUMING_LINE_TRACE)
            
        elif current_state == IntersectionState.RESUMING_LINE_TRACE:
            # 라인 트레이싱 재개
            self.controller.start_line_tracing()
            # 다음 레시피 액션으로 이동
            self.next_action()
            # 완료 상태로 변경
            self.set_state(IntersectionState.COMPLETED)
            
        elif current_state == IntersectionState.COMPLETED:
            # 교차로 처리 완료, 다음 교차로 감지를 위해 IDLE로 돌아감
            if robot_state == RobotState.LINE_TRACING:
                time.sleep(2.0)  # 같은 교차로를 여러 번 감지하지 않도록 잠시 대기
                self.set_state(IntersectionState.IDLE)

    def start(self):
        """교차로 처리 시작"""
        self.get_logger().info('교차로 처리 시작')
        self.set_state(IntersectionState.IDLE)
        # 라인 트레이싱 시작
        self.controller.start_line_tracing()

def main(args=None):
    """메인 함수"""
    rclpy.init(args=args)
    
    try:
        # 노드 생성
        intersection_handler = IntersectionHandler()
        
        # 교차로 처리 시작
        intersection_handler.start()
        
        # 스핀
        rclpy.spin(intersection_handler)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 종료
        if 'intersection_handler' in locals():
            intersection_handler.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

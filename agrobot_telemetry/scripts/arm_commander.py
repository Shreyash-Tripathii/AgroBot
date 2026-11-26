#!/usr/bin/env python3
"""
arm_commander: MoveIt plan/execute as plain ROS services, for the Foxglove "Arm Planner" panel.

Foxglove can call services but not actions, so this wraps move_group:

  /arm_commander/plan     moveit_msgs/srv/GetMotionPlan  fills defaults (group, start state, pipeline,
                          planning time), calls /plan_kinematic_path, remembers the trajectory
  /arm_commander/execute  std_srvs/srv/Trigger           runs the remembered plan via /execute_trajectory
  /arm_commander/stop     std_srvs/srv/Trigger           cancels a running execution
  /arm_commander/status   std_msgs/String (latched)      idle | planning | planned | executing |
                                                          succeeded | failed: <reason>
"""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import MoveItErrorCodes, RobotState
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

ERROR_NAMES = {v: k for k, v in vars(MoveItErrorCodes).items()
               if k.isupper() and isinstance(v, int)}


def code_name(code):
    return ERROR_NAMES.get(code, str(code))


class ArmCommander(Node):
    def __init__(self):
        super().__init__('arm_commander')
        self.declare_parameter('group', 'arm')
        self.declare_parameter('pipeline', 'ompl')
        self.declare_parameter('planner_id', '')
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('attempts', 5)

        cb = ReentrantCallbackGroup()
        self._cb = cb
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path', callback_group=cb)
        self.exec_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory', callback_group=cb)

        self.joint_state = JointState()
        self.create_subscription(JointState, '/joint_states', self._on_js, 10, callback_group=cb)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '/arm_commander/status', latched)

        self.create_service(GetMotionPlan, '/arm_commander/plan', self._plan, callback_group=cb)
        self.create_service(Trigger, '/arm_commander/execute', self._execute, callback_group=cb)
        self.create_service(Trigger, '/arm_commander/stop', self._stop, callback_group=cb)

        self.plan = None          # RobotTrajectory
        self.goal_handle = None
        self._status('idle')

    def _status(self, text):
        self.get_logger().info(f'status: {text}')
        self.status_pub.publish(String(data=text))

    def _on_js(self, msg):
        self.joint_state = msg

    # ------------------------------------------------------------------ plan
    def _plan(self, request, response):
        req = request.motion_plan_request
        if not self.plan_client.wait_for_service(timeout_sec=1.0):
            self._status('failed: move_group not available')
            response.motion_plan_response.error_code.val = MoveItErrorCodes.FAILURE
            return response
        if not req.group_name:
            req.group_name = self.get_parameter('group').value
        if not req.pipeline_id:
            req.pipeline_id = self.get_parameter('pipeline').value
        if not req.planner_id:
            req.planner_id = self.get_parameter('planner_id').value
        if req.allowed_planning_time <= 0.0:
            req.allowed_planning_time = float(self.get_parameter('planning_time').value)
        if req.num_planning_attempts <= 0:
            req.num_planning_attempts = int(self.get_parameter('attempts').value)
        if not req.start_state.joint_state.name:
            req.start_state = RobotState()
            req.start_state.joint_state = self.joint_state
        req.start_state.is_diff = False
        if req.max_velocity_scaling_factor <= 0.0:
            req.max_velocity_scaling_factor = 0.5
        if req.max_acceleration_scaling_factor <= 0.0:
            req.max_acceleration_scaling_factor = 0.5

        self._status('planning')
        self.plan = None
        fut = self.plan_client.call_async(GetMotionPlan.Request(motion_plan_request=req))
        # reentrant group + multithreaded executor: wait without blocking other callbacks
        deadline = self.get_clock().now().nanoseconds + int((req.allowed_planning_time + 5.0) * 1e9)
        while not fut.done():
            if self.get_clock().now().nanoseconds > deadline:
                self._status('failed: planning timed out')
                response.motion_plan_response.error_code.val = MoveItErrorCodes.TIMED_OUT
                return response
            time.sleep(0.02)
        result = fut.result()
        response.motion_plan_response = result.motion_plan_response
        code = result.motion_plan_response.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self.plan = result.motion_plan_response.trajectory
            self._status('planned')
        else:
            self._status(f'failed: {code_name(code)}')
        return response

    # ------------------------------------------------------------------ execute
    def _execute(self, request, response):
        if self.plan is None:
            response.success, response.message = False, 'no plan: press Plan first'
            return response
        if self.goal_handle is not None:
            response.success, response.message = False, 'already executing'
            return response
        if not self.exec_client.wait_for_server(timeout_sec=1.0):
            self._status('failed: move_group not available')
            response.success, response.message = False, 'execute_trajectory action not available'
            return response
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = self.plan
        self._status('executing')
        send = self.exec_client.send_goal_async(goal)
        send.add_done_callback(self._on_goal_response)
        response.success, response.message = True, 'started'
        return response

    def _on_goal_response(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self._status('failed: execution rejected')
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut):
        self.goal_handle = None
        res = fut.result().result
        code = res.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self._status('succeeded')
        elif code == MoveItErrorCodes.PREEMPTED:
            self._status('failed: stopped')
        else:
            self._status(f'failed: {code_name(code)}')

    # ------------------------------------------------------------------ stop
    def _stop(self, request, response):
        if self.goal_handle is None:
            response.success, response.message = False, 'nothing is executing'
            return response
        self.goal_handle.cancel_goal_async()
        response.success, response.message = True, 'cancel requested'
        return response


def main():
    rclpy.init()
    node = ArmCommander()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()

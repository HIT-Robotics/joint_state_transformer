#!/usr/bin/env python3
from robot_model import RobotModel, calcValidMotionState, CoordinateType, levenbergMarquardt
import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np
import os

class JointStateTransformer(Node):

    def __init__(self):
        qos_robot_description = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        qos_joint_state = QoSProfile(depth=1, durability=QoSDurabilityPolicy.VOLATILE)
        super().__init__('joint_state_transformer')
        self.robot_model = None
        self.last_state = None
        self._joint_names = None
        self._last_seq = -1
        
        self.robot_description_subscription = self.create_subscription(
            String,
            '/robot_description',
            self.robot_description_callback,
            qos_robot_description)
        
        self.declare_parameter('max_iterations', 100)
        self.declare_parameter('tolerance', 1e-5)
        self.declare_parameter('max_step_line_search', 10)
        self.declare_parameter('input_action_name', '~/follow_joint_trajectory')
        self.declare_parameter('output_action_name', '/joint_trajectory_controller/follow_joint_trajectory')
        self.declare_parameter('input_joint_states_topic', '/joint_states')
        self.declare_parameter('output_joint_states_topic', '/joint_states')

        self.max_iter = self.get_parameter('max_iterations').value
        self.tolerance = self.get_parameter('tolerance').value
        self.max_step_line_search = self.get_parameter('max_step_line_search').value

        self.joint_state_publisher = self.create_publisher(JointState, self.get_parameter('output_joint_states_topic').value, qos_joint_state)

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            self.get_parameter('output_joint_states_topic').value,
            self.joint_state_callback,
            qos_joint_state)

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.get_parameter('input_action_name').value,
            self.execute_action_callback)
        
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.get_parameter('output_action_name').value)
        
        self.get_logger().info(f"Parameters: max_iterations={self.max_iter}, tolerance={self.tolerance}")

    def robot_description_callback(self, msg:String)->None:
        # prevent multiple initializations
        if self.initialized:
            return
        # initialize based on the recieved robot_description
        filename = '/tmp/joint_state_transformer/description.urdf' 
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        with open(filename,'w') as file:
            file.write(msg.data)
        self.robot_model = RobotModel(filename)
        self._joint_names = np.array(self.robot_model.joint_names)
        
        self.last_state, _ = calcValidMotionState(self.robot_model, tol=self.tolerance, num_iterations=self.max_iter)
            

    def joint_state_callback(self, msg:JointState)->None:
        if not self.initialized:
            return

        # check if message is valid for transformation
        if msg.header.stamp == self._last_seq:
            return
        
        self.get_logger().debug(f"Received joint state: {msg.name}")
        
        try:
            mask_int = self.robot_model.joints.getMask(msg.name)
        except Exception as e:
            self.get_logger().error(f"Failed to get mask for joints {msg.name}: {e}")
            return
            
        mask = mask_int.astype(bool)
        
        if len(msg.position) != mask.sum():
            self.get_logger().error(f"Message position length ({len(msg.position)}) does not match mask sum ({mask.sum()})")
            return

        state = self.last_state.copy()
        state[mask] = msg.position
        
        result, success = levenbergMarquardt(self.robot_model, mask_int, state, tol=self.tolerance, max_iterations=self.max_iter, max_step_line_search=self.max_step_line_search)
        
        if not success:
            self.get_logger().warn(f"Levenberg-Marquardt failed to converge for given positions")
            return
            
        self.last_state = result
        self._last_seq = msg.header.stamp
        
        ret_msg = JointState()
        ret_msg.header = msg.header
        ret_msg.name = list(self._joint_names)
        ret_msg.position = self.last_state
        self.joint_state_publisher.publish(ret_msg)
        
    async def execute_action_callback(self, goal_handle):
        self.get_logger().info('Executing trajectory transformation goal...')
        
        if not self.initialized:
            self.get_logger().error('Robot model not initialized')
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        goal = goal_handle.request
        input_trajectory = goal.trajectory
        
        try:
            mask_int = self.robot_model.joints.getMask(input_trajectory.joint_names)
        except Exception as e:
            self.get_logger().error(f"Failed to get mask for joints {input_trajectory.joint_names}: {e}")
            goal_handle.abort()
            return FollowJointTrajectory.Result()
            
        mask = mask_int.astype(bool)
        active_mask = self.robot_model.joints.active_joints.astype(bool)
        active_joint_names = list(self._joint_names[active_mask])
        
        output_trajectory = input_trajectory.__class__()
        output_trajectory.header = input_trajectory.header
        output_trajectory.joint_names = active_joint_names
        
        current_state = self.last_state.copy()

        self.get_logger().info(f"Transforming {len(input_trajectory.points)} points...")
        for point in input_trajectory.points:
            current_state[mask] = point.positions
            result, success = levenbergMarquardt(self.robot_model, mask_int, current_state, tol=self.tolerance, max_iterations=self.max_iter, max_step_line_search=self.max_step_line_search)
            
            if not success:
                self.get_logger().error(f"Levenberg-Marquardt failed to converge during trajectory transformation")
                goal_handle.abort() 
                return FollowJointTrajectory.Result()
            
            current_state = result
            new_point = JointTrajectoryPoint()
            new_point.positions = list(current_state[active_mask])
            new_point.time_from_start = point.time_from_start
            # Note: Velocities and accelerations are not transformed as it requires Jacobian calculations.
            output_trajectory.points.append(new_point)
            
        # Forward to target controller
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Target action server not available')
            goal_handle.abort()
            return FollowJointTrajectory.Result()
            
        forward_goal = FollowJointTrajectory.Goal()
        forward_goal.trajectory = output_trajectory
        forward_goal.multi_dof_trajectory = goal.multi_dof_trajectory
        forward_goal.path_tolerance = goal.path_tolerance
        forward_goal.goal_tolerance = goal.goal_tolerance
        forward_goal.goal_time_tolerance = goal.goal_time_tolerance
        
        self.get_logger().info('Forwarding transformed trajectory to target controller...')
        send_goal_future = self.action_client.send_goal_async(
            forward_goal,
            feedback_callback=lambda feedback: goal_handle.publish_feedback(feedback.feedback))
            
        # Wait for goal response
        forward_goal_handle = await send_goal_future
        if not forward_goal_handle.accepted:
            self.get_logger().error('Target controller rejected the goal')
            goal_handle.abort()
            return FollowJointTrajectory.Result()
            
        # Wait for result
        get_result_future = forward_goal_handle.get_result_async()
        result = await get_result_future
        
        if result.status == GoalStatus.STATUS_SUCCEEDED:
             self.get_logger().info('Target controller succeeded')
             goal_handle.succeed()
        elif result.status == GoalStatus.STATUS_CANCELED:
             self.get_logger().info('Target controller canceled')
             goal_handle.canceled()
        else:
             self.get_logger().info(f'Target controller failed with status {result.status}')
             goal_handle.abort()
             
        return result.result
        

    @property
    def initialized(self)->bool:
        return self.robot_model is not None



def main(args=None):
    # get description
    print('Hi from joint_state_transformer.')
    rclpy.init(args=args)

    joint_state_transformer = JointStateTransformer()

    try:
        rclpy.spin(joint_state_transformer)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joint_state_transformer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

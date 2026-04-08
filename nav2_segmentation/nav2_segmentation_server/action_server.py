from __future__ import annotations

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from nav2_segmentation.action import SegmentImage


class Nav2SegmentationServer(Node):
    def __init__(self) -> None:
        super().__init__('nav2_segmentation')
        self.declare_parameter('action_name', 'segment_image')
        self.declare_parameter('default_mask_topic', '/segmentation/mask')

        action_name = self.get_parameter('action_name').get_parameter_value().string_value
        self._default_mask_topic = (
            self.get_parameter('default_mask_topic').get_parameter_value().string_value
        )

        self._action_server = ActionServer(
            self,
            SegmentImage,
            action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(f'Started dummy segmentation action server on {action_name}')

    def goal_callback(self, goal_request: SegmentImage.Goal) -> GoalResponse:
        self.get_logger().info(
            'Received goal '
            f'image_topic={goal_request.image_topic} '
            f'text_prompt={goal_request.text_prompt} '
            f'use_point_prompt={goal_request.use_point_prompt} '
            f'point=({goal_request.point_x}, {goal_request.point_y}) '
            f'point_label={goal_request.point_label} '
            f'use_box_prompt={goal_request.use_box_prompt} '
            f'box=({goal_request.box_min_x}, {goal_request.box_min_y}, '
            f'{goal_request.box_max_x}, {goal_request.box_max_y})'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        self.get_logger().info('Received cancel request for segmentation goal')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle) -> SegmentImage.Result:
        feedback = SegmentImage.Feedback()
        feedback.current_state = 'dummy_preparing'
        goal_handle.publish_feedback(feedback)

        time.sleep(0.05)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = SegmentImage.Result()
            result.success = False
            result.mask_topic = ''
            result.message = 'Dummy segmentation goal cancelled.'
            return result

        feedback.current_state = 'dummy_complete'
        goal_handle.publish_feedback(feedback)

        result = SegmentImage.Result()
        result.success = True
        result.mask_topic = self._default_mask_topic
        result.message = 'Dummy segmentation response for discussion-only PR.'
        goal_handle.succeed()
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Nav2SegmentationServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

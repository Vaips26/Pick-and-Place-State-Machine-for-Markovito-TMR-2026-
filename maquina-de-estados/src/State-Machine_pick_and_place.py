#!/usr/bin/env python3

from typing import re
import rospy
from collections import Counter
import json
import sys
from basicmodutil_pkg import commBM
import time
from transitions import Machine, State

from rospy import msg
from std_msgs.msg import String
from std_msgs.msg import Duration
from std_msgs.msg import Bool
from std_msgs.msg import Time
from std_msgs.msg import Float64
from module_simple_det.msg import ObjectInfoArray
from module_simple_det.msg import ObjectInfoYoloArray
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2

from arm_movement_planning.msg import manipulation_grasp_object_msg as grasp_msg
from arm_movement_planning.msg import manipulation_place_object_msg as place_msg
from arm_movement_planning.msg import manipulation_move_arm_to_pose_msg as pose_msg


class StoringGroceriesTaskCoordinator:
    """ROS-based task coordinator using transitions library for state management"""

    def __init__(self):
        # ===== SECTION 1: ROS PUBLISHERS AND SUBSCRIBERS =====
        top_comm_sub = '/function/output'
        top_event_sub = "/event/output"
        top_comm_pub = '/master/output'

        self.comm_pub = rospy.Publisher(top_comm_pub, String, queue_size=1)
        self.comm_sub = rospy.Subscriber(top_comm_sub, String, self.change_state)
        self.event_sub = rospy.Subscriber(top_event_sub, String, self.events_callback)

        self.pub_speak = rospy.Publisher("/pocket_listener/talk", String, queue_size=1)
        self.pub_move_base = rospy.Publisher("/movebase/position", String, queue_size=1)
        self.pub_arm_pose = rospy.Publisher("/module_arm_manipulation/move_arm_to_pose", pose_msg, queue_size=1)
        self.pub_arm_grasp = rospy.Publisher("/module_arm_manipulation/grasp_object", grasp_msg, queue_size=1)
        self.pub_arm_place = rospy.Publisher("/module_arm_manipulation/place_object", place_msg, queue_size=1)
        self.pub_arm_cartesian = rospy.Publisher("/module_arm_manipulation/move_to_cartesian_pose", Pose, queue_size=1)
        self.pub_grasp_aux_node = rospy.Publisher("/grasp_aux_node/command", String, queue_size=1)

        self.movebase_status_sub = rospy.Subscriber('/movebase/status', String, self.movebase_ready_callback)


        self.trash_offset_z = rospy.get_param('trash_offset_z', 0.25)
        self.dish_offset_z = rospy.get_param('dish_offset_z', 0.30)

        # ===== SECTION 2: ROS PARAMETERS =====
        # Objects related variables
        self.known_objects_classes = rospy.get_param('known_objects_classes').split(":")
        self.known_objects_categories = rospy.get_param('known_objects_categories').split(":")
        self.grasp_probabilities = [int(x) for x in rospy.get_param('grasp_probabilities').split(":")]
        self.detection_probabilities = [int(x) for x in rospy.get_param('detection_probabilities').split(":")]
        self.objects_to_be_detected = rospy.get_param('objects_to_be_detected').split(":")
        self.objects_to_be_detected_cathegory = rospy.get_param('objects_to_be_detected_cathegory').split(":")
        self.trash_can_name = rospy.get_param('trash_can_name','trash can')
        self.dishwasher_name = rospy.get_param('dishwasher_name','dish washer')
        self.object_global_index = 0
        self.trash_to_detect_index = 0 

        # Navigation wait-points names
        self.start_point = rospy.get_param('start_point', 'dining_table')
        self.table_point = rospy.get_param('table_point', 'dining_table')
        self.shelf_inspection_point = rospy.get_param('shelf_inspection_point', 'cabinet_inspection')
        self.shelf_deliver_point = rospy.get_param('shelf_deliver_point', 'cabinet_delivery')
        self.trashcan_point = rospy.get_param('trashcan_point', 'trash')
        self.dishwasher_point = rospy.get_param('dishwasher_point', 'dishwasher')
        self.move_back_distance = rospy.get_param('move_back_distance', -0.2)
        self.move_back_distance_2 = rospy.get_param('move_back_distance_2', -0.5)
        self.move_back_distance_trash = rospy.get_param('move_back_distance_trash', -0.2)
        self.move_back_distance_trash2 = rospy.get_param('move_back_distance_trash2', 0.1)
        self.optimal_distance_to_grasp = rospy.get_param('optimal_distance_to_grasp', 0.55)
        self.optimal_distance_to_grasp_floor = rospy.get_param('optimal_distance_to_grasp_floor', 0.4)

        # Torso and Head parameters for Object detection in Table
        self.torso_height_table = rospy.get_param('torso_height_table', 1.0)
        self.head_pan_table = rospy.get_param('head_pan_table', 0.0)
        self.head_tilt_table = rospy.get_param('head_tilt_table', 0.0)

        # Torso and Head parameters for shelf inspection
        self.torso_height_shelf_inspection = rospy.get_param('torso_height_shelf_inspection', 1.0)
        self.head_pan_shelf_inspection = rospy.get_param('head_pan_shelf_inspection', 0.0)
        self.head_tilt_shelf_inspection = rospy.get_param('head_tilt_shelf_inspection', 0.0)

        # Torso and Head parameters for place the objects in the shelf, level 1 (the lowest)
        self.torso_height_level_1 = rospy.get_param('torso_height_level_1', 1.0)
        self.head_pan_level_1 = rospy.get_param('head_pan_level_1', 0.0)
        self.head_tilt_level_1 = rospy.get_param('head_tilt_level_1', 0.6)

        # Torso and Head parameters for place the objects in the shelf, level 2 (the middle)
        self.torso_height_level_2 = rospy.get_param('torso_height_level_2', 0.5)
        self.head_pan_level_2 = rospy.get_param('head_pan_level_2', 0.0)
        self.head_tilt_level_2 = rospy.get_param('head_tilt_level_2', 0.6)

        # Torso and Head parameters for place the objects in the shelf, level 3 (the highest)
        self.torso_height_level_3 = rospy.get_param('torso_height_level_3', 0.1)
        self.head_pan_level_3 = rospy.get_param('head_pan_level_3', 0.0)
        self.head_tilt_level_3 = rospy.get_param('head_tilt_level_3', 0.6)

        # Torso and Head parameters for Navigation
        self.torso_height_nav = rospy.get_param('torso_height_nav', 0.1)
        self.head_pan_nav = rospy.get_param('head_pan_nav', 0.0)
        self.head_tilt_nav = rospy.get_param('head_tilt_nav', 0.0)

        # Torso and Head parameters for pickinf trash on the floor
        self.torso_height_floor = rospy.get_param('torso_height_floor', 0.1)
        self.head_pan_floor = rospy.get_param('head_pan_floor', 0.0)
        self.head_tilt_floor = rospy.get_param('head_tilt_floor', 0.75)

        # Tolerance for position adjustment
        self.tolerance_to_not_adjust = rospy.get_param('tolerance_to_not_adjust', 0.02)

        # Arm related parameters
        self.home_pose = rospy.get_param('home_pose', 'home_initial')
        self.home_normal_pose = rospy.get_param('home_normal_pose', 'home_normal')
        self.out_pose = rospy.get_param('out_pose', 'home_out')
        self.out_pose_downwards = rospy.get_param('out_pose_downwards', 'home_out_downwards')
        self.arm_gripper_release = rospy.get_param('arm_gripper_release', 'gripper_release')
        self.arm_gripper_descendreel = rospy.get_param('arm_gripper_descendreel', 'gripper_descendreel')
        self.arm_gripper_ascend = rospy.get_param('arm_gripper_ascend', 'gripper_ascend')
        self.level_1_pose = rospy.get_param('level_1_pose', 'level_1_c')
        self.level_2_pose = rospy.get_param('level_2_pose', 'level_2_c')
        self.level_3_pose = rospy.get_param('level_3_pose', 'level_3_c')
        self.shelf_release_pose = rospy.get_param('shelf_release_pose', 'release_table')
        self.arm_out_time_out = rospy.get_param('arm_out_time_out', 5)
        self.arm_gripper_descendreel = rospy.get_param('arm_gripper_descendreel', 'gripper_descendreel')
        self.arm_gripper_ascend = rospy.get_param('arm_gripper_ascend', 'gripper_ascend')

        self.level_1_category = rospy.get_param('level_1_category', 'empty')
        self.level_2_category = rospy.get_param('level_2_category', 'empty')
        self.level_3_category = rospy.get_param('level_3_category', 'empty')
        self.manual_categorization = rospy.get_param('manual_categorization', False)

        # Shelf levels delimiters in Pixels in the image taken from the shelf inspection point
        self.y1 = rospy.get_param('y1', 50)
        self.y2 = rospy.get_param('y2', 100)
        self.y3 = rospy.get_param('y3', 150)
        # Shelf width delimiters in Pixels in the image taken from the shelf inspection point
        self.x_left = rospy.get_param('x_left', 180)
        self.x_right = rospy.get_param('x_right', 360)
        # Shelf x delimiters for place ce the objects in left, center or right in Pixels in the image taken from the shelf inspection point
        self.x_delimiters = [int(x) for x in rospy.get_param('x_delimiters').split(":")]

        # Dynamic variables
        self.not_graspable = rospy.get_param('not_graspable_objects').split(":")  # List of objects that are not graspable
        self.grasped_object_class = ""  # Class of the grasped object
        self.grasped_object_category = "food"  # Category of the grasped object
        self.level_category = [self.level_1_category, self.level_2_category, self.level_3_category]
        self.arm_torso_level_pose = self.level_2_pose
        self.trash_objects = [
            obj for obj, cat in zip(self.objects_to_be_detected, self.objects_to_be_detected_cathegory)
            if cat == 'trash'
        ]

        # ===== SECTION 3: STATE MACHINE VARIABLES =====
        self._last_result = None  # Store ROS callback success/failure
        self._last_ros_msgs = None  # Store full ROS message data

        # TODO UNCOMMENT THIS TO RUN THE REAL TASK
        initial_state = 'no_action'

        # For testing purposes TODO COMMENT THIS TO RUN THE REAL TASK
        initial_state = 'moving_to_trashcan_point'

        # ===== SECTION 4: TRANSITIONS SETUP =====
        states = self._define_states()

        # Initialize machine
        self.machine = Machine(
            model=self,
            states=states,
            initial=initial_state,
            auto_transitions=False,
            ignore_invalid_triggers=True
        )

        # Define transitions
        self._setup_transitions()

    # ===== SECTION 5: STATE DEFINITIONS =====
    def _define_states(self):
        """Define all active states with entry callbacks"""
        return [
            'no_action',
            'waiting_open_door',
            'moving_to_start_point',
            'arm_out',
            'torso_and_head_adjust_1',
            'moving_to_table',
            'detecting_objects_on_table',
            'detecting_objects_on_table_post_adjust',
            'detecting_objects_on_table_final',
            'moving_to_breakfast_table',
            'taking_object',
            'arm_out_with_obj',
            'arm_out_no_obj',
            'moving_step_back',
            'arm_home_with_obj',
            'torso_and_head_adjust_2',
            'moving_to_trashcan_point',
            'torso_and_head_adjust_trashcan',
            'move_arm_out_trashcan',
            'detecting_trashcan',
            'adjusting_to_trashcan_position',
            'detecting_trashcan_2',
            'move_arm_to_trashcan_centroid',
            'releasing_gripper_on_trashcan',
            'moving_step_back_trashcan',
            'arm_out_after_trash',
            'detecting_objects_on_floor',
            'adjust_position_object_floor',
            'detecting_again_objects_on_floor',
            'taking_object_on_floor',
            'arm_out_obj_floor',
            'arm_out_no_obj_floor',
            'detecting_trashcan_floor',
            'adjusting_to_trashcan_position_2',
            'adjusting_to_trashcan_position_2_fail',
            'detecting_trashcan_floor2',
            'move_arm_to_trashcan_centroid_floor',
            'releasing_gripper_on_trashcan_floor',
            'arm_home_after_floor',
            'detecting_trashcan_obj_floor',
            'adjusting_to_trashcan_position_3',
            'adjusting_to_trashcan_position_3_fail',
            'detecting_trashcan_floor3',
            'move_arm_to_trashcan_centroid_floor2',
            'releasing_gripper_on_trashcan_floor2',
            'navigate_to_table_after_floor', 
            'torso_head_adjust_table_floor',
            'moving_to_dishwasher_point',
            'torso_and_head_adjust_dishwasher',
            'move_arm_out_dishwasher',
            'detecting_dishwasher',
            'adjusting_to_dishwasher_position',
            'adjusting_to_dishwasher_position_fail',
            'detecting_dishwasher_2',
            'move_arm_to_dishwasher_centroid',
            'releasing_gripper_dishwasher',
            'ascending_gripper_dishwasher',
            'arm_out_after_dishwasher',
            'moving_back_dishwasher',
            'arm_home_after_dishwasher',
            'torso_and_head_adjust_after_dishwasher',
            'moving_to_shelf_inspection',
            'end_task',
        ]

    # ===== SECTION 6: TRANSITION DEFINITIONS =====
    def _setup_transitions(self):
        """Define all state transitions"""

        # STEP 0: Start task
        self.machine.add_transition(
            trigger='start_task',
            source='no_action',
            dest='waiting_open_door',
            after=[
                'wait_for_door_opening',
                lambda: self.give_feedback("Task started... Waiting for door to be opened...")
            ]
        )

        self.machine.add_transition(
            trigger='stop_task',
            source='*',
            dest='no_action'
        )

        # STEP 1: Door opening → Navigate to start
        self.machine.add_transition(
            trigger='action_completed',
            source='waiting_open_door',
            dest='moving_to_start_point',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Door is open, navigating to the safe kitchen point."),
                lambda: self.navigate_to_wait_point(self.start_point)
            ]
        )

        # STEP 2: Navigate to start (success/failure paths)
        self.machine.add_transition(
            trigger='action_completed',
            source='moving_to_start_point',
            dest='arm_out',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Navigation completed. Moving the arm to out position."),
                lambda: self.move_arm_to_pose(self.out_pose)
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='moving_to_start_point',
            dest='moving_to_start_point',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Navigation fail, trying again."),
                lambda: self.navigate_to_wait_point(self.start_point)
            ]
        )

        # STEP 3: Arm out (success/failure paths)
        self.machine.add_transition(
            trigger='action_completed',
            source='arm_out',
            dest='torso_and_head_adjust_1',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Arm move completed. Adjusting Torso and Head."),
                lambda: self.move_torso_pan_tilt(self.torso_height_table, self.head_pan_table, self.head_tilt_table)
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='arm_out',
            dest='arm_out',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Arm move fail, trying again."),
                lambda: self.move_arm_to_pose(self.out_pose)
            ]
        )

        # STEP 4: Torso adjust → Navigate to table
        self.machine.add_transition(
            trigger='action_completed',
            source='torso_and_head_adjust_1',
            dest='moving_to_table',
            after=[
                lambda: self.give_feedback("Torso Adjustment completed. Navigating to the table inspection point."),
                lambda: self.navigate_to_wait_point(self.table_point)
            ]
        )

        # STEP 5: Navigate to table (success/failure paths)
        self.machine.add_transition(
            trigger='action_completed',
            source='moving_to_table',
            dest='detecting_objects_on_table',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Navigation completed. Detecting objects on the table."),
                lambda: self.detect_objects(self.objects_to_be_detected[self.object_global_index])
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='moving_to_table',
            dest='moving_to_table',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Navigation fail, trying again."),
                lambda: self.navigate_to_wait_point(self.table_point)
            ]
        )

        # STEP 6: Object detection (complex branching)
        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_objects_on_table',
            dest='detecting_objects_on_table_post_adjust',
            conditions='_object_detected',
            after='_calculate_position_adjustment'
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_objects_on_table',
            dest='detecting_objects_on_table',
            conditions=['_no_object_detected', '_has_more_objects'],
            after='_try_next_object'
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_objects_on_table',
            dest='moving_to_breakfast_table',
            conditions=['_no_object_detected', '_no_more_objects'],
            after=[
                lambda: self.give_feedback("I have found no objects... Moving to the service table..."),
                lambda: self.navigate_to_wait_point(self.breakfast_point)
            ]
        )

        # STEP 7: After position adjustment → Re-detect
        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_objects_on_table_post_adjust',
            dest='detecting_objects_on_table_final',
            after=[
                lambda: self.give_feedback("Finishing the adjustment"),
                lambda: self.give_feedback(f"The object to grasp is: {self.grasped_object_class}. " +
                                  f"The object category is: {self.grasped_object_category}"),
                lambda: self.detect_objects(self.objects_to_be_detected[self.object_global_index])
            ]
        )

        # STEP 8: Final detection after adjustment
        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_objects_on_table_final',
            dest='taking_object',
            conditions='_is_success',
            after='_grasp_detected_object'
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_objects_on_table_final',
            dest='detecting_objects_on_table',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Detecting again."),
                lambda: self.detect_objects(self.objects_to_be_detected[self.object_global_index])
            ]
        )

        # STEP 9: Grasp object (success/failure paths)
        self.machine.add_transition(
            trigger='action_completed',
            source='taking_object',
            dest='arm_out_with_obj',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Object grasped. Moving the arm to the out pose."),
                lambda: self.move_arm_to_pose(self.out_pose)
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='taking_object',
            dest='arm_out_no_obj',
            conditions='_is_failure',
            after=['_mark_object_ungraspable', lambda: self.move_arm_to_pose(self.out_pose)]
        )

        # STEP 10: Arm out with object
        self.machine.add_transition(
            trigger='action_completed',
            source='arm_out_with_obj',
            dest='moving_step_back',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Arm move completed. Step back."),
                lambda: self.move_base_in_meters(self.move_back_distance)
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='arm_out_with_obj',
            dest='arm_out_with_obj',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Arm move fail, trying again."),
                lambda: self.move_arm_to_pose(self.out_pose)
            ]
        )

        # Arm out without object (retry detection)
        self.machine.add_transition(
            trigger='action_completed',
            source='arm_out_no_obj',
            dest='detecting_objects_on_table',
            conditions='_is_success',
            after=lambda: self.detect_objects(self.objects_to_be_detected[self.object_global_index])
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='arm_out_no_obj',
            dest='arm_out_no_obj',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Arm move fail, trying again."),
                lambda: self.move_arm_to_pose(self.out_pose)
            ]
        )

        # STEP 11: Step back
        self.machine.add_transition(
            trigger='action_completed',
            source='moving_step_back',
            dest='arm_home_with_obj',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Step back completed. Moving the arm to the home pose."),
                lambda: self.move_arm_to_pose(self.home_pose)
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='moving_step_back',
            dest='moving_step_back',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Fail to move back, trying again."),
                lambda: self.move_base_in_meters(self.move_back_distance)
            ]
        )

        # STEP 12: Arm home with object
        self.machine.add_transition(
            trigger='action_completed',
            source='arm_home_with_obj',
            dest='torso_and_head_adjust_2',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Arm move completed. Adjusting Torso and Head."),
                lambda: self.move_torso_pan_tilt(self.torso_height_nav, self.head_pan_nav, self.head_tilt_nav)
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='arm_home_with_obj',
            dest='arm_home_with_obj',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Arm move fail, trying again."),
                lambda: self.move_arm_to_pose(self.home_pose)
            ]
        )

        # STEP 13: Deciding where to go depending on the object cathegory
        #########  BRANCH TRASH ############
        
        # STEP 13T:
        self.machine.add_transition(
            trigger='action_completed',
            source='torso_and_head_adjust_2',
            dest='moving_to_trashcan_point',
            conditions='_is_trash_object',
            after=[
                lambda: self.give_feedback("Moving to the Trash Can Point."),
                lambda: self.navigate_to_wait_point(self.trashcan_point)
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='moving_to_trashcan_point',
            dest='moving_to_trashcan_point',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Trash Can Point not reached... Trying again"),
                lambda: self.navigate_to_wait_point(self.trashcan_point)
            ]
        )
        

        # STEP 14T: Moving the head down to detect trashbin
        self.machine.add_transition(
            trigger='start_task', # trigger='action_completed',
            source='moving_to_trashcan_point',
            dest='torso_and_head_adjust_trashcan',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Trashcan point reached... Adjusting head..."),
                lambda: self.move_torso_pan_tilt(self.torso_height_floor, self.head_pan_floor, self.head_tilt_floor)
            ]
        )

                # STEP 14D: Llegó al dishwasher — ajustar cabeza
        self.machine.add_transition(
            trigger='action_completed',
            source='moving_to_dishwasher_point',
            dest='torso_and_head_adjust_dishwasher',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Dishwasher point reached. Adjusting head."),
                lambda: self.move_torso_pan_tilt(self.torso_height_nav, self.head_pan_nav, self.head_tilt_table)
            ]
        )






        # STEP 15T: Arm out (success/failure paths)
        self.machine.add_transition(
            trigger='action_completed',
            source='torso_and_head_adjust_trashcan',
            dest='move_arm_out_trashcan',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Head adjusted... Driving the arm to the out position..."),
                lambda: self.move_arm_to_pose(self.out_pose)
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='move_arm_out_trashcan',
            dest='move_arm_out_trashcan',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Arm move fail, trying again."),
                lambda: self.move_arm_to_pose(self.out_pose)
            ]
        )

        # STEP 16T: Detecting trashcan
        self.machine.add_transition(
            trigger='action_completed',
            source='move_arm_out_trashcan',
            dest='detecting_trashcan',
            conditions='_is_success',
            after= [
                lambda: self.give_feedback("Detecting trash bin"),
                lambda: self.detect_objects(self.trash_can_name)
            ]
        )

        # STEP 17T
        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_trashcan',
            dest='adjusting_to_trashcan_position',
            conditions='_object_detected',
            after='_calculate_position_adjustment'
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_trashcan',
            dest='move_arm_out_trashcan',
            conditions='_no_object_detected',
            after=[
                lambda: self.give_feedback("No trashcan detected, trying again"),
                lambda: self.move_base_in_meters(self.move_back_distance_trash2)
            ]
        )

        # STEP 18T: Detecting trashcan
        self.machine.add_transition(
            trigger='action_completed',
            source='adjusting_to_trashcan_position',
            dest='detecting_trashcan_2',
            conditions='_is_success',
            after=lambda: self.detect_objects(self.trash_can_name)
        )

        # STEP 19T: Send arm to trashbin centroid with offset
        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_trashcan_2',
            dest='move_arm_to_trashcan_centroid',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Moving arm to the trashcan"),
                '_move_arm_to_trashcan_centroid'
            ]
        )

        # STEP 20T: Release gripper
        self.machine.add_transition(
            trigger='action_completed',
            source='move_arm_to_trashcan_centroid',
            dest='releasing_gripper_on_trashcan',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Releasing gripper"),
                lambda: self.move_arm_to_pose(self.arm_gripper_release)
            ]
        )

        # STEP 21T: Move back 20 centimeters
        self.machine.add_transition(
            trigger='action_completed',
            source='releasing_gripper_on_trashcan',
            dest='moving_step_back_trashcan',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Gripper releasing, stepping back."),
                lambda: self.move_base_in_meters(self.move_back_distance_trash)
            ]
        )
        
        # STEP 22T: Arm out again
        self.machine.add_transition(
            trigger='action_completed',
            source='moving_step_back_trashcan',
            dest='arm_out_after_trash',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Step back done. Moving arm to out pose"),
                lambda: self.move_arm_to_pose(self.out_pose_downwards)
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='arm_out_after_trash',
            dest='arm_out_after_trash',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Arm move fail, trying again."),
                lambda: self.move_arm_to_pose(self.out_pose_downwards)
            ]
            
        )

        # STEP 23T: Detecting trash on the floor
        self.machine.add_transition(
            trigger='action_completed',
            source='arm_out_after_trash',   
            dest='detecting_objects_on_floor',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Arm out ready, Detecting objects on the floor."),
                lambda: self.detect_objects(self.trash_objects[self.trash_to_detect_index])
            ]
        )

        # STEP 24T: Adjusting to pick up the trash on the floor
        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_objects_on_floor',
            dest='adjust_position_object_floor',
            conditions='_object_detected',
            after='_calculate_position_adjustment_floor'
        )
        
        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_objects_on_floor',
            dest='detecting_objects_on_floor',
            conditions=['_no_object_detected', '_has_more_trash_objects'],
            after='_try_next_trash_object'
        )

        self.machine.add_transition(
           trigger='action_completed',
           source='detecting_objects_on_floor',
           dest='moving_to_table',
           conditions=['_no_object_detected', '_no_more_objects'],
           after=[
               lambda: self.give_feedback("I have found no more trash... Moving to table..."),
                lambda: self.navigate_to_wait_point(self.table_point)
           ]
         )

        # STEP 25T: Detecting again trash on the floor
        self.machine.add_transition(
            trigger='action_completed',
            source='adjust_position_object_floor',   
            dest='detecting_again_objects_on_floor',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Arm out ready, Detecting objects on the floor."),
                lambda: self.detect_objects(self.trash_objects[self.trash_to_detect_index])
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_again_objects_on_floor',
            dest='moving_to_table',
            conditions='_no_object_detected',
            after=lambda: self.navigate_to_wait_point(self.table_point)
        )

        # STEP 26T: Grasping the trash on the floor
        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_again_objects_on_floor',
            dest='taking_object_on_floor',
            conditions='_is_success',
            after='_grasp_detected_object'
        )

        # STEP 27T: Arm_out Grasp object (success/failure paths)
        self.machine.add_transition(
            trigger='action_completed',
            source='taking_object_on_floor',
            dest='arm_out_obj_floor',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Object grasped from the floor. Moving the arm to the out pose."),
                lambda: self.move_arm_to_pose(self.out_pose)
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='arm_out_obj_floor',
            dest='arm_out_obj_floor',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Arm out fail. Trying again"),
                lambda: self.move_arm_to_pose(self.out_pose)
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='taking_object_on_floor',
            dest='arm_out_no_obj_floor',
            conditions='_is_failure',
            after=['_mark_object_ungraspable', lambda: self.move_arm_to_pose(self.out_pose)]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='arm_out_no_obj_floor',
            dest='detecting_objects_on_floor',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Arm out. Trying to detect again."),
                lambda: self.detect_objects(self.trash_objects[self.trash_to_detect_index])
            ]
        )
        
        self.machine.add_transition(
            trigger='action_completed',
            source='arm_out_no_obj_floor',
            dest='arm_out_no_obj_floor',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Arm move fail, trying again."),
                lambda: self.move_arm_to_pose(self.out_pose)
             ]
        )

        # STEP 28T: Detecting trashcan again for trash
        self.machine.add_transition(
            trigger='action_completed',
            source='arm_out_obj_floor',
            dest='detecting_trashcan_floor',
            conditions='_is_success',
            after= [
                lambda: self.give_feedback("Detecting trash bin"),
                lambda: self.detect_objects(self.trash_can_name)
            ]
        )

        #  STEP 29T: Adjusting to trashcan 2
        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_trashcan_floor',
            dest='adjusting_to_trashcan_position_2',
            conditions='_object_detected',
            after='_calculate_position_adjustment'
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_trashcan_floor',
            dest='adjusting_to_trashcan_position_2_fail',
            conditions='_no_object_detected',
            after=[
                lambda: self.give_feedback("No trashcan detected, trying again"),
                lambda: self.move_base_in_meters(self.move_back_distance_trash2)
            ]
        )

        #moverse y volver a intentar 
        self.machine.add_transition(
            trigger='action_completed',
            source='adjusting_to_trashcan_position_2_fail',
            dest='detecting_trashcan_floor',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Trying to detect trashcan again."),
                lambda: self.detect_objects(self.trash_can_name)
            ]
        )

         # STEP 30T: Detecting trashcan floor 2
        self.machine.add_transition(
            trigger='action_completed',
            source='adjusting_to_trashcan_position_2',
            dest='detecting_trashcan_floor2',
            conditions='_is_success',
            after=[lambda: self.detect_objects(self.trash_can_name)
            ]
        )
        
        #STEP 31: Go to centroid position
        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_trashcan_floor2',
            dest='move_arm_to_trashcan_centroid_floor',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Moving arm to the trashcan"),
                '_move_arm_to_trashcan_centroid'
            ]
        )

        # STEP 32T: Release gripper
        self.machine.add_transition(
            trigger='action_completed',
            source='move_arm_to_trashcan_centroid_floor',
            dest='releasing_gripper_on_trashcan_floor',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Releasing gripper"),
                lambda: self.move_arm_to_pose(self.arm_gripper_release)
            ]
        )
          
        # STEP 33T: Arm home after floor release
        self.machine.add_transition(
            trigger='action_completed',
            source='releasing_gripper_on_trashcan_floor',
            dest='arm_home_after_floor',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Gripper released. Moving arm to home."),
                lambda: self.move_arm_to_pose(self.home_pose)
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='arm_home_after_floor',
            dest='arm_home_after_floor',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Arm move fail, trying again."),
                lambda: self.move_arm_to_pose(self.home_pose)
            ]
        )

        #STEP 34T Detecting trashcan 3 floor
        self.machine.add_transition(
            trigger='action_completed',
            source='arm_home_after_floor',
            dest='detecting_trashcan_obj_floor',
            conditions='_is_success',
            after= [
                lambda: self.give_feedback("Detecting trash bin"),
                lambda: self.detect_objects(self.trash_can_name)
            ]
        )

        #  STEP 35T: Adjusting to trashcan 3 from floor
        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_trashcan_obj_floor',
            dest='adjusting_to_trashcan_position_3',
            conditions='_object_detected',
            after='_calculate_position_adjustment'
        )
        
        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_trashcan_floor',
            dest='adjusting_to_trashcan_position_3_fail',
            conditions='_no_object_detected',
            after=[
                lambda: self.give_feedback("No trashcan detected, trying again"),
                lambda: self.move_base_in_meters(self.move_back_distance_trashcan)
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='adjusting_to_trashcan_position_3_fail',
            dest='detecting_trashcan_floor3',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Trying to detect trashcan again."),
                lambda: self.detect_objects(self.trash_can_name)
            ]
        )

         #  STEP 36: Go to centroid position
        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_trashcan_floor3',
            dest='move_arm_to_trashcan_centroid_floor2',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Moving arm to the trashcan"),
                '_move_arm_to_trashcan_centroid'
            ]
        )

        # STEP 37T: Release gripper
        self.machine.add_transition(
            trigger='action_completed',
            source='move_arm_to_trashcan_centroid_floor2',
            dest='releasing_gripper_on_trashcan_floor2',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Releasing gripper"),
                lambda: self.move_arm_to_pose(self.arm_gripper_release)
            ]
        )


        # STEP 38T: Move back 20 centimeters
        self.machine.add_transition(
            trigger='action_completed',
            source='releasing_gripper_on_trashcan_floor2',
            dest='moving_step_back_trashcan_floor2',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Gripper releasing, stepping back."),
                lambda: self.move_base_in_meters(self.move_back_distance_trash)
            ]
        )
          
        # STEP 34T: Arm home after floor release
        self.machine.add_transition(
            trigger='action_completed',
            source='moving_step_back_trashcan_floor2',
            dest='arm_home_after_floor2',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Step back complete. Moving arm to home."),
                lambda: self.move_arm_to_pose(self.home_pose)
            ]
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='arm_home_after_floor2',
            dest='arm_home_after_floor2',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Arm move fail, trying again."),
                lambda: self.move_arm_to_pose(self.home_pose)
            ]
        )

        # STEP 35T: Move to the table point again
        
        self.machine.add_transition(
            trigger='action_completed',
            source='arm_home_after_floor',
            dest='moving_to_table',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Trash dropping complete. Moving back to the table."),
                lambda: self.navigate_to_wait_point(self.table_point)
            ]
        )
 

        #########  BRANCH DISH ############      

        # STEP 13D: Moving to the dishwasher if not reached the first time
        self.machine.add_transition(
            trigger='action_completed',
            source='moving_to_dishwasher_point',
            dest='moving_to_dishwasher_point',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Door is open, navigating to the safe kitchen point."),
                lambda: self.start_parallel_actions([   
                    lambda: self.navigate_to_point(self.dishwasher_point),
                    lambda: self.move_arm_to_pose(self.out_pose_downwards)
                    
                ]),
            ],
            dest='in_start_point',
        )

          # STEP 14D: Llegó al dishwasher — ajustar cabeza
        self.machine.add_transition(
            trigger='action_completed',
            source='moving_to_dishwasher_point',
            dest='torso_and_head_adjust_dishwasher',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Dishwasher point reached. Adjusting head."),
                lambda: self.move_torso_pan_tilt(self.torso_height_nav, self.head_pan_nav, self.head_tilt_table)
            ]
        )

         # STEP 15D: Detectar dishwasher
        self.machine.add_transition(
            source='moving_to_dishwasher_point',
            trigger='action_completed',
            dest='detecting_dishwasher',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Detecting dishwasher."),
                lambda: self.detect_objects(self.dishwasher_name)
            ]
        )


          # STEP 17D: Ajustar posicion
        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_dishwasher',
            dest='adjusting_to_dishwasher_position',
            conditions='_object_detected',
            after='_calculate_position_adjustment'
        )

        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_dishwasher',
            dest='adjusting_to_dishwasher_position_fail',
            conditions='_no_object_detected',
            after=[
                lambda: self.give_feedback("Dishwasher not detected, trying again."),
                lambda: self.detect_objects(self.dishwasher_name)
            ]
        )

          # STEP 18D: Detectar de nuevo
        self.machine.add_transition(
            trigger='action_completed',
            source='adjusting_to_dishwasher_position',
            dest='detecting_dishwasher_2',
            conditions='_is_success',
            after=lambda: self.detect_objects(self.dishwasher_name)
        )


        # STEP 19D: Mandar brazo al centroide
        self.machine.add_transition(
            trigger='action_completed',
            source='detecting_dishwasher_2',
            dest='move_arm_to_dishwasher_centroid',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Moving arm to dishwasher."),
                '_move_arm_to_dishwasher_centroid'
            ]
        )

        # STEP 20D: Descendreel
        self.machine.add_transition(
            trigger='action_completed',
            source='move_arm_to_dishwasher_centroid',
            dest='releasing_gripper_dishwasher',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Releasing object in dishwasher."),
                lambda: self.move_arm_to_pose(self.arm_gripper_descendreel)
            ]
        )

        # STEP 21D: Ascend
        self.machine.add_transition(
            trigger='action_completed',
            source='releasing_gripper_dishwasher',
            dest='ascending_gripper_dishwasher',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Ascending gripper."),
                lambda: self.move_arm_to_pose(self.arm_gripper_ascend)
            ]
        )

        # STEP 22D: Brazo a out_pose después de ascend
        self.machine.add_transition(
            trigger='action_completed',
            source='ascending_gripper_dishwasher',
            dest='arm_out_after_dishwasher',
            conditions='_is_success',
            after=[
                lambda: self.give_feedback("Ascend completed. Moving arm out."),
                lambda: self.move_arm_to_pose(self.out_pose)
            ]
        )




            # STEP 24D: Moving back in front of dishwasher
        self.machine.add_transition(
            trigger='action_completed',
            source='arm_out_after_dishwasher',
            dest='moving_back_dishwasher',
            after=[
                    lambda: self.give_feedback("Moving back in front of dishwasher."),
                    lambda: self.move_base_in_meters(self.move_back_distance)
                ]
            )

            # STEP 25D: Home initial in front of dishwasher
        self.machine.add_transition(
            trigger='action_completed',
            source='moving_back_dishwasher',
            dest='arm_home_after_dishwasher',
            after=[
                    lambda: self.give_feedback("Moving arm to home initial."),
                    lambda: self.move_arm_to_pose(self.home_pose)
                ]
            )
        


                # STEP 25D failure: retry home
        self.machine.add_transition(
            trigger='action_completed',
            source='arm_home_after_dishwasher',
            dest='arm_home_after_dishwasher',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Arm home failed, trying again."),
                lambda: self.move_arm_to_pose(self.home_pose)
            ]
        )




                # STEP 26D: Adjust torso and head for navigation
        self.machine.add_transition(
            trigger='action_completed',
            source='arm_home_after_dishwasher',
            dest='torso_and_head_adjust_after_dishwasher',
            conditions='_is_success',
            after=[
                    lambda: self.give_feedback("Adjusting torso and head for navigation."),
                    lambda: self.move_torso_pan_tilt(self.torso_height_nav, self.head_pan_nav, self.head_tilt_nav)
                    ]
                )

            # STEP 27D: Navigate to the table point again
        self.machine.add_transition(
                trigger='action_completed',
                source='torso_and_head_adjust_after_dishwasher',
                dest='moving_to_table',
                after=[
                    lambda: self.give_feedback("Navigating back to table."),
                    lambda: self.navigate_to_wait_point(self.table_point)
                ]
            )
        
        



        # STEP 14D: Head and torso adjustment in front of dishwasher (robot should ask to open the dishwasher here)

        # STEP 15D: Arm out in front of dishwasher

        # STEP 16D: Detect dishwasher

        # STEP 17D: Adjust depending on the dishwasher centroid

        # STEP 18D: Detecting again the dishwasher

        # STEP 19D: Moving the arm to the dishwasher dropping pose

        # STEP 20D: Moving the arm down in vertical motion (alternatively use grasp_aux node to drop)

        # STEP 21D: Releasing gripper on the dishwasher

        # STEP 22D: Moving the arm up in vertical motion

        # STEP 23D: Arm out in front of dishwasher post dropping

        # STEP 24D: Moving back in front of dishwasher

        # STEP 25D: Home initial in front of dishwasher

        # STEP 26D: Adjust torso and head for navigation

        # STEP 27D: Navigate to the table point again


        #########  BRANCH SHELF ############  

        # STEP 13S: Moving to the shelf inspection point if not reached the first time
        self.machine.add_transition(
            trigger='action_completed',
            source='moving_to_shelf_inspection',
            dest='moving_to_shelf_inspection',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Shelf Inspection Point not reached... Trying again"),
                lambda: self.navigate_to_wait_point(self.shelf_inspection_point)
            ]
        )

        # STEP 14S: Head and torso adjustment in front of shelf for inspecting levels

        # STEP 15S: Detecting location in 3D to place the current object according with its cathegory

        # STEP 16S: Substract certain distance (approx. 30cm) in the x axis and add approx. 5cm to the z axis of 
        # the placing point and adjust the base to 55cm of that point

        # STEP 17S: Drive the arm to that Cartesian place and with horizontal orientation, as if 
        # ready for dropping the object on the shelf

        # STEP 18S: Move base the distance above (approx. 30 centimeters) in x direction

        # STEP 19S: Use grasp_aux routine to drop the object on the shelf when detecting collision

        # STEP 20S: Move slighly the arm vertically (approx. 3cm)

        # STEP 21S: Step back some distance after dropping the object on the shelf

        # STEP 22S: Drive the arm to home after dropping the object on the shelf

        # STEP 23S: Adjust torso and head for navigation

        # STEP 24S: Navigate to the table point again

        ######## BRANCH BREAKFAST #########

        self.machine.add_transition(
            trigger='action_completed',
            source='moving_to_breakfast_table',
            dest='moving_to_breakfast_table',
            conditions='_is_failure',
            after=[
                lambda: self.give_feedback("Navigation to the service table failed. Trying again."),
                lambda: self.navigate_to_wait_point(self.breakfast_point)
            ]
        )

    # ===== SECTION 7: ROS CALLBACKS (ADAPTED) =====
    def events_callback(self, data):
        """ROS event callback - preserved signature"""
        mod_name, event_name, param_names, param_ros_msgs = commBM.readEvent2Ros(str(data.data))
        print("┌──────────────────┐")
        print("     ", mod_name)
        print("     ", event_name)
        print("└──────────────────┘")

        if event_name == "start_task":
            self.start_task()  # Trigger transition
        elif event_name == "stop_task":
            self.stop_task()  # Trigger transition

    def change_state(self, data):
        """ROS function output callback - adapted to dispatch to state machine"""
        names, ros_msgs = commBM.readMsg2Ros(data.data)

        print("┌────────────────────────────────────────────────┐")
        print(" ", names)
        print(" ", ros_msgs)
        print("  Current state:", self.state)

        # Store callback data
        self._last_result = ros_msgs[0].data if len(ros_msgs) > 0 else None
        self._last_ros_msgs = ros_msgs

        # Dispatch to state machine
        try:
            self.action_completed()
        except Exception as e:
            rospy.logerr(f"State transition error: {e}")

        print("  New state:", self.state)
        print("└────────────────────────────────────────────────┘")

    def movebase_ready_callback(self, msg):
        """ROS movebase status callback"""
        if msg.data == "ready":
            # All movebase movements complete with simulated success
            self.send_simulated_success_response()

    # ===== SECTION 8: TRANSITION CALLBACKS (Complex logic only) =====
    def _grasp_detected_object(self):
        """Grasp the detected object"""
        if self._last_ros_msgs and len(self._last_ros_msgs) > 1:
            detected_objs = self._last_ros_msgs[1].objects
            if detected_objs:
                self.grasp_object(detected_objs[0])

    def _calculate_position_adjustment(self):
        """Called when object is detected - calculate adjustment"""
        if self._last_ros_msgs and len(self._last_ros_msgs) > 1:
            detected_objs = self._last_ros_msgs[1].objects
            if detected_objs:
                
                self.grasped_object_class = self.objects_to_be_detected[self.object_global_index]
                self.grasped_object_category = self.objects_to_be_detected_cathegory[self.object_global_index]
                
                position_to_adjust = detected_objs[0].position_base.x - self.optimal_distance_to_grasp
                rospy.loginfo(f"Object position_base.x: {detected_objs[0].position_base.x}")

                # Only adjust if position difference is significant
                # if abs(position_to_adjust) > self.tolerance_to_not_adjust:
                self.give_feedback(f"Adjusting position: {position_to_adjust:.2f} meters.")
                self.move_base_in_meters(position_to_adjust)

                # else:
                    # Position is good enough, skip adjustment
                    # self.send_simulated_success_response()
    
    def _calculate_position_adjustment_floor(self):
        """Called when object is detected - calculate adjustment"""
        if self._last_ros_msgs and len(self._last_ros_msgs) > 1:
            detected_objs = self._last_ros_msgs[1].objects
            if detected_objs:
                
                self.grasped_object_class = self.objects_to_be_detected[self.object_global_index]
                self.grasped_object_category = self.objects_to_be_detected_cathegory[self.object_global_index]
                
                position_to_adjust = detected_objs[0].position_base.x - self.optimal_distance_to_grasp_floor
                rospy.loginfo(f"Object position_base.x: {detected_objs[0].position_base.x}")

                # Only adjust if position difference is significant
                # if abs(position_to_adjust) > self.tolerance_to_not_adjust:
                self.give_feedback(f"Adjusting position: {position_to_adjust:.2f} meters.")
                self.move_base_in_meters(position_to_adjust)

                # else:
                    # Position is good enough, skip adjustment
                    # self.send_simulated_success_response()

    def _try_next_object(self):
        """Called when object not detected - try next in list"""
        self.object_global_index = self.object_global_index + 1
        self.give_feedback("Detecting the next object.")
        self.detect_objects(self.objects_to_be_detected[self.object_global_index])

    def _try_next_trash_object(self):
        """Called when object not detected - try next in list"""
        self.trash_to_detect_index = self.trash_to_detect_index + 1
        self.give_feedback("Detecting the next trash.")
        self.detect_objects(self.trash_objects[self.trash_to_detect_index])
 
    def _mark_object_ungraspable(self):
        """Called when grasp fails - mark object as ungraspable"""
        self.not_graspable.append(self.grasped_object_class)
        self.give_feedback("Fail to grasp the object, trying with other.")

    def _move_arm_to_trashcan_centroid(self):
        if self._last_ros_msgs and len(self._last_ros_msgs) > 1:
            trash_obj = self._last_ros_msgs[1].objects
            offset_x = 0.0
            offset_y = 0.0
            offset_z = self.trash_offset_z

            target = Pose()
            target.position.x = trash_obj[0].position_base.x + offset_x
            target.position.y = trash_obj[0].position_base.y + offset_y
            target.position.z = trash_obj[0].position_base.z + offset_z  # offset seguridad

            # Orientación del gripper hacia abajo
            target.orientation.x =  0.7071
            target.orientation.y =  0.7071
            target.orientation.z =  0.0
            target.orientation.w =  0.0

            rospy.loginfo(f"[TRASH] Centroide BASE: x={trash_obj[0].position_base.x:.3f} "
                        f"y={trash_obj[0].position_base.y:.3f} z={trash_obj[0].position_base.z:.3f}")

            rospy.loginfo(f"[TRASH] Target con offset Z: x={target.position.x:.3f} "
                        f"y={target.position.y:.3f} z={target.position.z:.3f}")

            self.pub_arm_cartesian.publish(target)

    def _move_arm_to_dishwasher_centroid(self):
        if self._last_ros_msgs and len(self._last_ros_msgs) > 1:
            dish_obj = self._last_ros_msgs[1].objects
            offset_x = 0.0
            offset_y = 0.0
            offset_z = self.dish_offset_z

            target = Pose()
            target.position.x = dish_obj[0].position_base.x + offset_x
            target.position.y = dish_obj[0].position_base.y + offset_y
            target.position.z = dish_obj[0].position_base.z + offset_z

            target.orientation.x = 0.7071
            target.orientation.y = 0.7071
            target.orientation.z = 0.0
            target.orientation.w = 0.0

        self.pub_arm_cartesian.publish(target)

    def _release_gripper_dishwasher(self):
        self.pub_grasp_aux_node.publish("disreal")












   

    # ===== SECTION 9: CONDITION METHODS =====
    def _is_success(self):
        """Condition: Check if last action succeeded"""
        return self._last_result is True

    def _is_failure(self):
        """Condition: Check if last action failed"""
        return self._last_result is False

    def _object_detected(self):
        """Condition: Check if object was detected"""
        return (self._last_result is True and
                self._last_ros_msgs and
                len(self._last_ros_msgs) > 1 and
                hasattr(self._last_ros_msgs[1], 'objects') and
                len(self._last_ros_msgs[1].objects) > 0)

    def _no_object_detected(self):
        """Condition: Check if detection failed"""
        return self._last_result is False

    def _has_more_objects(self):
        """Condition: Check if more objects to try"""
        return self.object_global_index < len(self.objects_to_be_detected) - 1

    def _no_more_objects(self):
        """Condition: Check if no more objects to try"""
        return self.object_global_index >= len(self.objects_to_be_detected) - 1

    def _is_trash_object(self):
        """Condition: Check if grasped object is trash"""
        return self.objects_to_be_detected_cathegory[self.object_global_index] == 'trash'

    def _has_more_trash_objects(self):
        return self.trash_to_detect_index < len(self.trash_objects) - 1

    def _no_more_trash_objects(self):
        return self.trash_to_detect_index >= len(self.trash_objects) - 1
 
    def _is_dish_object(self):
        """Condition: Check if grasped object is a dish"""
        return self.objects_to_be_detected_cathegory[self.object_global_index] == 'dish'

    # ===== SECTION 10: ROS ACTION METHODS (UNCHANGED) =====
    def place_object(self):
        ros_msg = place_msg()
        ros_msg.object_name = String(self.grasped_object_class)
        ros_msg.place_name = String(self.shelf_release_pose)
        ros_msg.place_point = ""
        ros_msg.time_out = Duration(rospy.Duration(self.arm_out_time_out))
        self.pub_arm_place.publish(ros_msg)

    def grasp_object(self, to_grasp_object):
        ros_msg = grasp_msg()
        ros_msg.object_name = to_grasp_object.obj_class
        ros_msg.object_pointCloud = to_grasp_object.pcd_path
        ros_msg.object_centroid = to_grasp_object.position
        ros_msg.time_out = Duration(rospy.Duration(self.arm_out_time_out))
        self.pub_arm_grasp.publish(ros_msg)

    def detect_objects(self, detection_place):
        t = String(detection_place)
        ros_msg = [t]
        js = commBM.writeFunCallFromRos('module_sam_det', 'objectsOnArea', ros_msg)
        msg = String(js)
        self.comm_pub.publish(msg)

    def move_torso_pan_tilt(self, torso_height, head_pan, head_tilt):
        torso_p = Float64(torso_height)
        pan_p = Float64(head_pan)
        tilt_p = Float64(head_tilt)
        ros_msg = [torso_p, pan_p, tilt_p]
        js = commBM.writeFunCallFromRos('module_torso_head', 'moveTorsoPanTilt', ros_msg)
        msg = String(js)
        self.comm_pub.publish(msg)

    def move_arm_to_pose(self, arm_pose_name):
        ros_msg = pose_msg()
        ros_msg.pose_name = arm_pose_name
        ros_msg.time_out = Duration(rospy.Duration(self.arm_out_time_out))
        self.pub_arm_pose.publish(ros_msg)

    def navigate_to_wait_point(self, wait_point_name):
        p = String(wait_point_name)
        ros_msg = [p]
        js = commBM.writeFunCallFromRos('NAV_MAP_BM', 'GoTo', ros_msg)
        msg = String(js)
        self.comm_pub.publish(msg)

    def wait_for_door_opening(self):
        print ("llamo a waiting for door opening")
        t = Float64(5000)
        ros_msg = [t]
        js = commBM.writeFunCallFromRos('NAV_MAP_BM', 'DetectDoorOpeningUp', ros_msg)
        msg = String(js)
        self.comm_pub.publish(msg)

    def give_feedback(self, feedback_message):
        print(feedback_message)
        print("└────────────────────────────────────────────────┘")
        self.pub_speak.publish(feedback_message)

    def move_base_in_meters(self, dist):
        ros_msg = str(dist)
        self.pub_move_base.publish(ros_msg)

    def send_simulated_success_response(self):
        """Modified to use new callback dispatch"""
        print("ento aqui")
        self._last_result = True
        self._last_ros_msgs = [Bool(True)]
        try:
            self.action_completed()
        except:
            pass

    def adjust_arm_pose_based_on_blank_space(self, level_objects, level_name):
        place_pose = "level_ " + str(level_name) + "_c"

        empty_spaces = [True, True, True]
        for obj in level_objects:
            x_position = obj.position2D.x
            if self.x_delimiters[0] < x_position < self.x_delimiters[1]:
                empty_spaces[0] = False
            elif self.x_delimiters[1] < x_position < self.x_delimiters[2]:
                empty_spaces[1] = False
            elif self.x_delimiters[2] < x_position < self.x_delimiters[3]:
                empty_spaces[2] = False

        if empty_spaces[0]:
            return "level_" + str(level_name) + "_l"
        elif empty_spaces[1]:
            return "level_ " + str(level_name) + "_c"
        elif empty_spaces[2]:
            return "level_" + str(level_name) + "_r"

        return place_pose


def main(args):
    rospy.init_node('storing_groceries_coordinator', anonymous=False)
    cord = StoringGroceriesTaskCoordinator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
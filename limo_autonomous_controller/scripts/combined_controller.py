#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
import time


class CombinedController:
    def __init__(self):
        rospy.init_node("combined_controller_node", anonymous=True)

        # Publishers and Subscribers
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/camera/rgb/image_raw/compressed", CompressedImage, self.camera_callback)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/path_", String, self.path_callback)

        # Data holders and flags
        self.bridge = CvBridge()
        self.cmd_msg = Twist()
        self.latest_image = None
        self.lidar_msg = None
        self.lidar_flag = False
        self.camera_flag = False
        self.path_command = None  # path_ 메시지 저장

        # Detection settings
        self.angle_resolution = math.pi / 640
        self.last_detection_time = time.time()
        self.previous_point = 290  # 초기 중앙값
        self.scan_degree = 45  # 좌우 45도
        self.min_dist = 0.39  # 39cm 거리
        self.default_speed = 0.25

        self.rate = rospy.Rate(20)  # 20Hz 작업 루프

    def camera_callback(self, msg):
        self.latest_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        self.camera_flag = True

    def lidar_callback(self, msg):
        self.lidar_msg = msg
        self.lidar_flag = True

    def path_callback(self, msg):
        """Callback for path_ topic."""
        self.path_command = msg.data
        rospy.loginfo(f"Received path command: {self.path_command}")

    def detect_lane(self):
        """Detect lanes using camera image."""
        if not self.camera_flag or self.latest_image is None:
            return None

        image = self.latest_image.copy()
        crop_image = image[360:480, :]  # 하단 120픽셀만 사용

        hsv = cv2.cvtColor(crop_image, cv2.COLOR_BGR2HSV)

        # 노란색과 흰색 검출
        yellow_mask = cv2.inRange(hsv, np.array([15, 80, 80]), np.array([45, 255, 255]))  # na.array(15,채도, 밝기)
        white_mask = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 30, 255]))
        mask = cv2.bitwise_or(yellow_mask, white_mask)

        simplify_image = cv2.bitwise_and(crop_image, crop_image, mask=mask)
        gray_img = cv2.cvtColor(simplify_image, cv2.COLOR_BGR2GRAY)
        binary_line = np.zeros_like(gray_img)
        binary_line[gray_img > 0] = 1

        histogram = np.sum(binary_line, axis=0)
        indices = np.where(histogram > 10)[0]

        # 디버깅용 이미지 출력
        try:
            cv2.imshow("Simplified Image (Lane Detection)", simplify_image)
            cv2.imshow("Grayscale Image (Lane Detection)", gray_img)
            cv2.waitKey(1)  # 화면 갱신
        except Exception as e:
            rospy.logerr(f"OpenCV Display Error: {e}")

        if len(indices) > 0:
            # 라인을 감지한 경우
            middle_point = int((min(indices) + max(indices)) / 2)
            self.previous_point = middle_point
            self.last_detection_time = time.time()
            return middle_point
        else:
            return None

    def detect_obstacle(self):
        """Detect obstacles using LiDAR data."""
        if not self.lidar_flag or self.lidar_msg is None:
            return False, 0.0

        degree_min = self.lidar_msg.angle_min * 180 / math.pi
        degree_increment = self.lidar_msg.angle_increment * 180 / math.pi
        degrees = [degree_min + degree_increment * i for i in range(len(self.lidar_msg.ranges))]

        obstacle_detected = False
        left_clearance, right_clearance = 0, 0

        for index, dist in enumerate(self.lidar_msg.ranges):
            if 0 < dist < self.min_dist:
                angle = degrees[index]
                if -self.scan_degree <= angle <= self.scan_degree:
                    obstacle_detected = True
                    if angle > 0:  # Right side
                        right_clearance += 1
                    elif angle < 0:  # Left side
                        left_clearance += 1

        # Calculate steering based on left/right obstacle density
        if obstacle_detected:
            if right_clearance > left_clearance:
                return True, -0.45  # Turn left
            else:
                return True, 0.45  # Turn right
        else:
            return False, 0.0

    def ctrl(self):
        """Control logic for lane keeping, obstacle avoidance, and V2X commands."""
        lane_center = self.detect_lane()
        obstacle_detected, obstacle_steer = self.detect_obstacle()

        flag = 0

        if obstacle_detected:
            # 장애물 회피
            print("Obstacle detected. Avoiding...")
            self.cmd_msg.linear.x = 0.15  # 장애물 회피 속도
            self.cmd_msg.angular.z = obstacle_steer
        elif lane_center is not None:
            # 라인 인식 우선
            print("Lane detected. Following lane...")
            error = 320 - lane_center
            self.cmd_msg.linear.x = self.default_speed
            self.cmd_msg.angular.z = -error * self.angle_resolution
        elif (self.path_command is not None and flag == 0):
            # V2X 명령 수행
            print(f"Executing V2X command: {self.path_command}")
            if self.path_command == "A":
                self.cmd_msg.linear.x = 0.25  # 0.18 #
                self.cmd_msg.angular.z = 0.25  # 0.18 # 왼쪽
                flag = 1
            elif self.path_command == "B":
                self.cmd_msg.linear.x = 0.20
                self.cmd_msg.angular.z = -0.12  # 중앙
                flag = 1
            elif self.path_command == "C":
                self.cmd_msg.linear.x = 0.17
                self.cmd_msg.angular.z = -0.21  # 오른쪽
                flag = 1
        else:
            # 차선 미탐지 및 V2X 명령 없음 - 기본 탐색
            print("No lane detected. Searching...")
            self.cmd_msg.linear.x = 0.18  # 기본 탐색 속도
            self.cmd_msg.angular.z = 0.18  # 기본 탐색 회전

        # Publish the command
        print(f"Linear X: {self.cmd_msg.linear.x}, Angular Z: {self.cmd_msg.angular.z}")
        self.pub.publish(self.cmd_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.ctrl()
            self.rate.sleep()


if __name__ == "__main__":
    controller = CombinedController()
    controller.run()

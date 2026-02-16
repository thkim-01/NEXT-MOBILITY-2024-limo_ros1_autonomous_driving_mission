#!/usr/bin/env python3

"""
V2X 경로 명령 테스트 스크립트
/path_ 토픽으로 명령을 보내 로봇의 반응을 테스트합니다.
"""

import rospy
from std_msgs.msg import String
import sys


def send_path_command(command):
    """V2X 경로 명령 전송"""
    rospy.init_node('v2x_test_publisher', anonymous=True)
    pub = rospy.Publisher('/path_', String, queue_size=10)
    
    # 퍼블리셔 초기화 대기
    rospy.sleep(1)
    
    msg = String()
    msg.data = command
    
    rospy.loginfo(f"V2X 명령 전송: {command}")
    pub.publish(msg)
    
    # 메시지 전송 확인
    rospy.sleep(0.5)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("사용법: rosrun limo_autonomous_controller test_v2x.py [A|B|C]")
        print("  A: 좌회전")
        print("  B: 직진")
        print("  C: 우회전")
        sys.exit(1)
    
    command = sys.argv[1].upper()
    
    if command not in ['A', 'B', 'C']:
        print(f"오류: 잘못된 명령 '{command}'")
        print("유효한 명령: A (좌회전), B (직진), C (우회전)")
        sys.exit(1)
    
    try:
        send_path_command(command)
        print(f"명령 '{command}' 전송 완료")
    except rospy.ROSInterruptException:
        pass

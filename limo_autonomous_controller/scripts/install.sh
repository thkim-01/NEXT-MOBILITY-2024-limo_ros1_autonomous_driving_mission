#!/bin/bash

# Limo Pro 자율주행 시스템 설치 스크립트
# 작성자: KIM TAE HEON
# NEXT MOBILITY 2024

echo "================================================"
echo "Limo Pro 자율주행 시스템 설치 시작"
echo "================================================"

# ROS 배포판 확인
if [ -z "$ROS_DISTRO" ]; then
    echo "오류: ROS 환경이 설정되지 않았습니다."
    echo "다음 명령어로 ROS 환경을 설정하세요:"
    echo "source /opt/ros/noetic/setup.bash"
    exit 1
fi

echo "감지된 ROS 배포판: $ROS_DISTRO"

# 의존성 패키지 설치
echo ""
echo "의존성 패키지 설치 중..."
sudo apt-get update

# ROS 패키지 설치
sudo apt-get install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-sensor-msgs \
    ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-std-msgs

# Python 패키지 설치
sudo apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-numpy

# catkin 워크스페이스 확인
if [ -z "$CATKIN_WS" ]; then
    CATKIN_WS=~/catkin_ws
fi

echo ""
echo "catkin 워크스페이스: $CATKIN_WS"

# 워크스페이스 생성 (없을 경우)
if [ ! -d "$CATKIN_WS/src" ]; then
    echo "catkin 워크스페이스 생성 중..."
    mkdir -p $CATKIN_WS/src
    cd $CATKIN_WS
    catkin_make
fi

# 스크립트 실행 권한 부여
echo ""
echo "실행 권한 부여 중..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
chmod +x $SCRIPT_DIR/scripts/combined_controller.py

# 빌드
echo ""
echo "패키지 빌드 중..."
cd $CATKIN_WS
catkin_make

# setup.bash 소싱
source $CATKIN_WS/devel/setup.bash

echo ""
echo "================================================"
echo "설치 완료!"
echo "================================================"
echo ""
echo "실행 방법:"
echo "  roslaunch limo_autonomous_controller autonomous_drive.launch"
echo ""
echo "문제가 발생하면 다음 명령어로 환경을 다시 로드하세요:"
echo "  source $CATKIN_WS/devel/setup.bash"
echo "================================================"

# NEXT-MOBILITY-2024-limo_ros1_autonomous_driving_mission
2024 넥스트모빌리티 자율주행 미션 코드

## 프로젝트 개요
WeGo Robotics Limo Pro 로봇을 위한 자율주행 제어 시스템입니다.
- **차선 추종**: 카메라 기반 노란색/흰색 차선 인식
- **장애물 회피**: LiDAR 기반 장애물 감지 및 회피
- **V2X 통신**: 외부 경로 명령 수신 및 실행

## 시스템 요구사항
- ROS Noetic (Ubuntu 20.04) 또는 ROS Melodic (Ubuntu 18.04)
- Python 3
- OpenCV
- NumPy
- cv_bridge

## 패키지 구조
```
limo_autonomous_controller/
├── CMakeLists.txt          # CMake 빌드 설정
├── package.xml             # ROS 패키지 메타데이터
├── scripts/
│   └── combined_controller.py  # 메인 제어 노드
└── launch/
    └── autonomous_drive.launch # 실행 launch 파일
```

## 설치 방법

### 1. 워크스페이스로 이동
```bash
cd ~/catkin_ws/src
```

### 2. 저장소 클론
```bash
git clone https://github.com/thkim-01/NEXT-MOBILITY-2024-limo_ros1_autonomous_driving_mission.git
cd NEXT-MOBILITY-2024-limo_ros1_autonomous_driving_mission
```

### 3. 의존성 설치
```bash
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-image-transport \
    python3-opencv \
    python3-numpy
```

### 4. 빌드
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 5. 실행 권한 부여
```bash
chmod +x ~/catkin_ws/src/NEXT-MOBILITY-2024-limo_ros1_autonomous_driving_mission/limo_autonomous_controller/scripts/combined_controller.py
```

## 실행 방법

### Launch 파일로 실행
```bash
roslaunch limo_autonomous_controller autonomous_drive.launch
```

### 단독 노드 실행
```bash
rosrun limo_autonomous_controller combined_controller.py
```

## ROS Topics

### Subscribe (구독)
- `/camera/rgb/image_raw/compressed` (sensor_msgs/CompressedImage): 압축된 카메라 이미지
- `/scan` (sensor_msgs/LaserScan): LiDAR 스캔 데이터
- `/path_` (std_msgs/String): V2X 경로 명령 (A, B, C)

### Publish (발행)
- `/cmd_vel` (geometry_msgs/Twist): 로봇 속도 제어 명령

## V2X 명령어
- **A**: 좌회전 경로
- **B**: 직진 경로
- **C**: 우회전 경로

## 파라미터 조정
코드 내부에서 다음 파라미터들을 조정할 수 있습니다:

```python
self.scan_degree = 45       # LiDAR 스캔 각도 (좌우 45도)
self.min_dist = 0.39        # 장애물 감지 최소 거리 (39cm)
self.default_speed = 0.25   # 기본 주행 속도
```

## 주의사항
1. Limo Pro의 카메라 및 LiDAR 드라이버가 먼저 실행되어야 합니다.
2. topic 이름이 다를 경우 코드에서 수정이 필요합니다.
3. GUI 환경에서 실행 시 차선 감지 디버깅 창이 표시됩니다.

## 문제 해결

### 카메라 토픽을 찾을 수 없음
```bash
rostopic list | grep camera
# 실제 토픽 이름을 확인하고 코드에서 수정하세요
```

### LiDAR 토픽을 찾을 수 없음
```bash
rostopic list | grep scan
# 실제 토픽 이름을 확인하고 코드에서 수정하세요
```

### OpenCV 디스플레이 오류
SSH 연결 시 X11 forwarding 설정 필요:
```bash
ssh -X user@robot_ip
```

## 개발자
- **KIM TAE HEON**
- NEXT MOBILITY 2024

## 라이선스
MIT License

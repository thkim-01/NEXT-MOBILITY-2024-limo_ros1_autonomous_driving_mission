# Limo Pro 자율주행 - 빠른 시작 가이드

## 1단계: 설치

```bash
# 워크스페이스로 이동 (또는 생성)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# 저장소 클론 (이미 클론했다면 생략)
git clone https://github.com/thkim-01/NEXT-MOBILITY-2024-limo_ros1_autonomous_driving_mission.git

# 자동 설치 스크립트 실행
cd NEXT-MOBILITY-2024-limo_ros1_autonomous_driving_mission/limo_autonomous_controller
./scripts/install.sh
```

## 2단계: Limo Pro 센서 확인

### 카메라 및 LiDAR 드라이버 실행
```bash
# 터미널 1: Limo Pro 기본 드라이버 실행
# (Limo Pro의 실제 launch 파일로 대체하세요)
roslaunch limo_bringup limo_start.launch
```

### 토픽 확인
```bash
# 터미널 2: 토픽 진단
rosrun limo_autonomous_controller check_topics.sh
```

필수 토픽이 모두 활성화되어 있어야 합니다:
- ✓ `/camera/rgb/image_raw/compressed`
- ✓ `/scan`
- ✓ `/cmd_vel`

## 3단계: 자율주행 시스템 실행

```bash
# 터미널 3: 자율주행 컨트롤러 실행
roslaunch limo_autonomous_controller autonomous_drive.launch
```

## 4단계: V2X 테스트 (선택사항)

```bash
# 터미널 4: V2X 명령 전송
# 좌회전
rosrun limo_autonomous_controller test_v2x.py A

# 직진
rosrun limo_autonomous_controller test_v2x.py B

# 우회전
rosrun limo_autonomous_controller test_v2x.py C
```

## 문제 해결

### 문제 1: 카메라 토픽을 찾을 수 없음
```bash
# 실제 카메라 토픽 확인
rostopic list | grep camera

# 코드에서 토픽 이름 수정
# scripts/combined_controller.py 파일의 22번째 줄 수정:
# rospy.Subscriber("/실제/토픽/이름", CompressedImage, self.camera_callback)
```

### 문제 2: LiDAR 토픽을 찾을 수 없음
```bash
# 실제 LiDAR 토픽 확인
rostopic list | grep scan

# 코드에서 토픽 이름 수정
# scripts/combined_controller.py 파일의 23번째 줄 수정:
# rospy.Subscriber("/실제/토픽/이름", LaserScan, self.lidar_callback)
```

### 문제 3: 로봇이 움직이지 않음
```bash
# cmd_vel 토픽으로 수동 명령 테스트
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

# 로봇이 움직이면 -> 컨트롤러 노드 문제
# 로봇이 안 움직이면 -> 모터 드라이버 문제
```

## 속도 조정

파라미터 파일 수정:
```bash
nano ~/catkin_ws/src/NEXT-MOBILITY-2024-limo_ros1_autonomous_driving_mission/limo_autonomous_controller/config/controller_params.yaml
```

또는 코드에서 직접 수정:
```bash
nano ~/catkin_ws/src/NEXT-MOBILITY-2024-limo_ros1_autonomous_driving_mission/limo_autonomous_controller/scripts/combined_controller.py
```

## 로그 확인

```bash
# 노드 로그 확인
rosnode list
rosnode info /combined_controller_node

# 토픽 데이터 확인
rostopic echo /cmd_vel
rostopic echo /scan
rostopic echo /path_
```

## 비상 정지

Ctrl+C를 눌러 노드를 종료하면 로봇이 정지합니다.

또는:
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

## 도움말

더 자세한 정보는 다음 문서를 참조하세요:
- [README.md](../README.md) - 전체 문서
- [ARCHITECTURE.md](ARCHITECTURE.md) - 시스템 아키텍처

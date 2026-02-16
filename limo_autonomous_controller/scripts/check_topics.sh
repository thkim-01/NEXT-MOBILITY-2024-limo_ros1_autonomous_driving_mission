#!/bin/bash

# Limo Pro 토픽 진단 스크립트
# 필요한 토픽이 활성화되어 있는지 확인합니다.

echo "=========================================="
echo "Limo Pro ROS 토픽 진단"
echo "=========================================="
echo ""

# 필요한 토픽 목록
REQUIRED_TOPICS=(
    "/camera/rgb/image_raw/compressed"
    "/scan"
    "/cmd_vel"
)

# 각 토픽 확인
for topic in "${REQUIRED_TOPICS[@]}"; do
    if rostopic list | grep -q "^$topic$"; then
        echo "✓ $topic : 활성"
        
        # 토픽 정보 표시
        echo "  타입: $(rostopic type $topic)"
        echo "  Hz: $(timeout 2 rostopic hz $topic 2>&1 | grep 'average rate' || echo '  데이터 없음')"
        echo ""
    else
        echo "✗ $topic : 비활성"
        echo ""
    fi
done

# V2X 토픽 확인 (선택적)
echo "선택적 토픽:"
if rostopic list | grep -q "^/path_$"; then
    echo "✓ /path_ : 활성 (V2X 통신)"
else
    echo "✗ /path_ : 비활성 (V2X 통신 없음)"
fi

echo ""
echo "=========================================="
echo "전체 토픽 목록:"
echo "=========================================="
rostopic list
echo ""

echo "특정 토픽의 데이터를 확인하려면:"
echo "  rostopic echo /topic_name"
echo ""

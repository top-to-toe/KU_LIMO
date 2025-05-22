FROM osrf/ros:noetic-desktop-full

# 컨테이너에 필요한 패키지들 설치
RUN apt update && apt install -y \
    git \
    nano \
    # 필요한 다른 도구들이 있다면 여기에 추가 (예: htop, tree 등)
    # vim \
    # net-tools \
    # iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# 이 아래로 추가 설정이 필요하면 넣을 수 있습니다.
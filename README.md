---
# 2025-03-15
---
- 개발 가상 환경 설치 및 설정(VMware pw: 11)
  - ubuntu 22.04 
    - (https://releases.ubuntu.com/jammy) - Desktop image
  - VMware 17 설치
    - https://drive.google.com/file/d/1twlHYAgrWeLSQRO_vHy68lJxr-n1qIWl/view?usp=sharing
  - git 복제(clone) 시 명령어 - git clone (git repository URL) . < 마지막에 한칸 띄우고 마침표 붙여줘야 현재 디렉토리에 복제본 끌어오겠다는 의미
    - 초기 세팅시 git config user.name = "(이름)"과 git config user.email = "(이메일주소)" 명령어를 통해 초기화해야함.


---
# 2025-05-31
---
## ROS 2 Humble 기반 SLAM (Cartographer 활용) 요약

### 1. 필요한 환경 설정 및 패키지 설치

* **환경 변수 설정:**

    ```bash
    (~/.bashrc)
    export TURTLEBOT3_MODEL=burger # 또는 다른 터틀봇3 모델 (waffle, etc.)
    source /usr/share/gazebo/setup.bash # Gazebo 사용을 위한 설정
    export SVGA_VGPU10=0 # 가상 GPU 관련 설정 (필요한 경우)
    alias killgazebo='pkill -9 gzserver; pkill -9 gzclient; pkill -9 gzweb; pkill -9 gzbridge' # Gazebo 관련 프로세스 종료 alias
    ```

* **필수 패키지 설치:**

    ```bash
    sudo apt install ros-humble-gazebo-* # Gazebo 관련 패키지
    sudo apt install ros-humble-turtlebot3-msgs # 터틀봇3 메시지 패키지
    sudo apt install ros-humble-turtlebot3-teleop # 터틀봇3 텔레오퍼레이션 패키지
    sudo apt install ros-humble-rqt-tf-tree # TF tree 시각화 툴
    sudo apt install ros-humble-cartographer # Cartographer SLAM 라이브러리
    sudo apt install ros-humble-cartographer-ros # Cartographer ROS 2 인터페이스
    sudo apt install ros-humble-navigation2 # Navigation2 패키지 (경로 계획, 장애물 회피 등)
    sudo apt install ros-humble-nav2-bringup # Navigation2 실행 관련 패키지
    ```

* **터틀봇3 시뮬레이션 관련 저장소 클론:**

    ```bash
    cd ~/kuLimo/colcon_ws/src
    git clone [https://github.com/agilexrobotics/limo_ros2.git](https://github.com/agilexrobotics/limo_ros2.git) # LIMO 로봇 관련
    git clone -b humble [https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git](https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git) # 터틀봇3 시뮬레이션
    git clone -b humble [https://github.com/ROBOTIS-GIT/DynamixelSDK.git](https://github.com/ROBOTIS-GIT/DynamixelSDK.git) # 다이나믹셀 SDK (터틀봇3 구동기 제어)
    git clone -b humble [https://github.com/ROBOTIS-GIT/turtlebot3.git](https://github.com/ROBOTIS-GIT/turtlebot3.git) # 터틀봇3 관련 핵심 패키지
    ```

* **워크스페이스 빌드:**

    ```bash
    cd ~/kuLimo/colcon_ws
    # limo_car 관련 디렉토리 생성 (수업 내용과 관련 없을 수 있음)
    cd ~/kuLimo/colcon_ws/src/limo_ros2/limo_car
    mkdir log
    mkdir worlds
    mkdir src
    cd ~/kuLimo/colcon_ws
    colcon build # 워크스페이스 빌드
    source install/setup.bash # 환경 설정
    ```

### 2. SLAM (Cartographer) 실행 및 운용

1.  **Gazebo 시뮬레이션 실행:**

    ```bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py use_sim_time:=True # use_sim_time 중요!
    ```

2.  **터틀봇3 텔레오퍼레이션 실행:**

    ```bash
    ros2 run turtlebot3_teleop teleop_keyboard
    ```

3.  **Cartographer 실행:**

    ```bash
    ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true # use_sim_time 중요!
    ```

4.  **rqt 실행 (선택적):**

    ```bash
    rqt
    ```

    * `rqt` 실행 후 `Plugins` -> `Visualization` -> `Node Graph`, `TF Tree` 등을 선택하여 노드 연결 관계나 TF tree를 시각적으로 확인할 수 있습니다.

5.  **SLAM 운용:**

    * 텔레오퍼레이션으로 터틀봇3를 움직여 Gazebo 환경의 지도를 Cartographer가 생성하는 것을 `rviz2` 창에서 확인.

### 3. 추가 정보

* `use_sim_time:=True` 는 시뮬레이션 환경에서 매우 중요한 설정.
  ROS 2에게 시뮬레이터의 가상 시간(simulation time)을 사용하도록 알려줌.
  이 설정이 없으면 시간이 어긋나서 센서 데이터 처리나 로봇 제어가 제대로 되지 않을 수 있음.
* `rqt`는 ROS 2 GUI 툴킷. `Node Graph`, `TF Tree` 외에도 다양한 플러그인을 제공하여 ROS 2 시스템을 모니터링하고 디버깅하는 데 유용.
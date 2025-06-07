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

    * 텔레오퍼레이션으로 터틀봇3를 움직여 Gazebo 환경의 지도를 Cartographer가 생성하는 것을 `rviz2` 창에서 확인합니다.

### 3. 추가 정보

* `use_sim_time:=True` 는 시뮬레이션 환경에서 매우 중요한 설정입니다. ROS 2에게 시뮬레이터의 가상 시간(simulation time)을 사용하도록 알려줍니다. 이 설정이 없으면 시간이 어긋나서 센서 데이터 처리나 로봇 제어가 제대로 되지 않을 수 있습니다.
* `rqt`는 ROS 2 GUI 툴킷입니다. `Node Graph`, `TF Tree` 외에도 다양한 플러그인을 제공하여 ROS 2 시스템을 모니터링하고 디버깅하는 데 유용합니다.


---
## ROS2 기반 로봇 시뮬레이션 및 SLAM 지도 구축 실습 정리
[**ROS2 기반의 로봇 시뮬레이션 환경 구축, URDF/XACRO를 이용한 로봇 모델링, 그리고 SLAM을 통한 지도 구축**]
이번 실습에서는 **ROS2** 환경에서 로봇 시뮬레이션을 진행하고 **SLAM(Simultaneous Localization and Mapping)**을 통해 지도를 구축하는 과정을 깊이 있게 다루었습니다. 특히, 로봇 모델링부터 시뮬레이션 환경 구축, 그리고 지도 저장까지 전반적인 워크플로우를 직접 경험하며 이해도를 높였습니다.

---

### 1. 시뮬레이션 환경 구축 및 패키지 설치

로봇 시뮬레이션을 위해 필수적인 패키지들을 설치하고 환경을 설정했습니다.

* **`rviz2`**: ROS2 시뮬레이션 데이터를 시각화하는 데 사용되는 강력한 도구입니다. 로봇의 상태, 센서 데이터, 지도 등을 실시간으로 확인할 수 있습니다.
* **`gazebo`**: 로봇 시뮬레이션을 위한 3D 물리 엔진입니다. 실제 로봇과 유사한 환경에서 다양한 시나리오를 테스트할 수 있습니다.
* **`urdf_launch`**: URDF(Unified Robot Description Format) 파일을 기반으로 로봇 모델을 시뮬레이션 환경에 로드하고 관련 노드를 실행하는 데 활용됩니다.

---

### 2. URDF (Unified Robot Description Format) 모델링

로봇 시뮬레이션의 핵심인 **URDF**를 직접 작성하며 로봇의 구조와 물리적 특성을 정의했습니다.

* **URDF 예제 작성**: 로봇의 링크(link)와 조인트(joint)를 정의하여 실제 로봇의 형상을 디지털로 표현하는 방법을 실습했습니다. 각 링크의 질량, 관성, 시각적 요소(mesh) 등을 설정했습니다.
* **URDF 유효성 검사 및 시각화**: 작성한 URDF 파일이 올바르게 정의되었는지 확인하기 위해 **`rqt_tf_tree`**를 사용했습니다. `rqt_tf_tree`는 로봇의 **tf(Transform)** 구조, 즉 각 링크 간의 상대적인 위치와 방향 관계를 시각적으로 보여주어 모델링 오류를 쉽게 파악할 수 있게 합니다.

---

### 3. XACRO (XML Macros)를 활용한 URDF 확장

복잡하고 반복적인 URDF 작성을 효율적으로 관리하기 위해 **XACRO**를 활용했습니다.

* **변수 및 매크로 사용 실습**: XACRO의 변수 기능을 사용하여 로봇의 크기나 재질과 같은 공통된 속성을 한 곳에서 관리하고, 매크로 기능을 통해 반복되는 로봇 부품(예: 바퀴, 센서)을 모듈화하여 코드의 가독성을 높이고 유지보수를 용이하게 했습니다. 이는 대규모 로봇 모델을 효율적으로 개발하는 데 필수적인 기법입니다.

---

### 4. SLAM (Simultaneous Localization and Mapping)을 통한 지도 구축

지난 시간에 이어 **SLAM**을 통해 로봇이 미지의 환경에서 자신의 위치를 추정함과 동시에 주변 환경의 지도를 구축하는 과정을 복습하고 심화했습니다.

* **SLAM 복습 및 지도 저장**: 로봇이 움직이면서 센서 데이터(예: LiDAR)를 기반으로 실시간으로 환경 지도를 생성하고, 생성된 지도를 영구적으로 저장하는 방법을 다시 한번 확인했습니다. 이 지도는 향후 로봇의 자율 주행이나 특정 임무 수행에 활용될 수 있습니다.

---
echo -e  "\e[32mInstalling enssential packages....\e[0m"
sudo apt-get update
sudo apt-get -y install --no-install-recommends \
ros-noetic-ros-control \
ros-noetic-ros-controllers \
ros-noetic-gazebo-ros \
ros-noetic-gazebo-ros-control \
ros-noetic-ros-controllers \
ros-noetic-joint-state-publisher-gui \
ros-noetic-teleop-twist-keyboard \
ros-noetic-joint-state-controller \
ros-noetic-move-base \
ros-noetic-robot-pose-ekf \
ros-noetic-gmapping \
ros-noetic-map-server \
ros-noetic-amcl \
ros-noetic-global-planner \
ros-noetic-teb-local-planner \
ros-noetic-rqt-robot-steering \
python-pip

# using gpproxy.com accelerate download
read -n1 -p "Do you want to download gazebo models? [y/n]" input
echo ""
if [ $input = "y" ];then
    echo -e "\e[32mNow dowloading gazebo models....\e[0m"
    git clone https://ghproxy.com/https://github.com/osrf/gazebo_models ~/.gazebo/models
fi

read -n1 -p "Do you want to install rosdepc and update rosdep database? [y/n]" input
echo ""
if [ $input = "y" ];then
    echo -e "\e[32mNow update rosdep database using rosdepc....\e[0m"
    sudo pip install rosdepc
    sudo rosdepc init
    rosdepc update
    rosdepc install --from-paths src --ignore-src -r -y
fi

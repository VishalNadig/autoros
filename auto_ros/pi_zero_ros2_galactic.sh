# Create Swap file memory because 512 MB RAM of Pi Zero is NOT ENOUGH
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo cp /etc/fstab /etc/fstab.bak
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
sudo sysctl vm.swappiness=70

locale  # check for UTF-8

sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
-o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
  
# install some pip packages needed for testing
python3 -m pip install -U \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  setuptools

sudo apt-get install libeigen3-dev
sudo apt install libssl-dev
sudo apt install libacl1-dev

mkdir -p ~/ros2_galactic/src
cd ~/ros2_galactic
wget https://raw.githubusercontent.com/ros2/ros2/galactic/ros2.repos
vcs import src < ros2.repos

sudo rosdep init
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-5.3.1 urdfdom_headers"

# Ignore GUI libraries
touch src/ros2/rviz/AMENT_IGNORE
touch src/ros-visualization/AMENT_IGNORE
touch src/ros2/system_tests/AMENT_IGNORE

export MAKEFLAGS=-j1
colcon build --symlink-install --executor sequential --packages-skip-build-finished >> ~/colcon_build_ros2.log

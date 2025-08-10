FROM ubuntu:22.04

LABEL maintainer="Kornel Mrozowski kornel.mrozowski@gmail.com"

ARG REMOTE_USER
ARG REMOTE_UID
ARG REMOTE_GID

RUN apt update && \
    apt upgrade -y && \
# tzdata
    ln -fs /usr/share/zoneinfo/Europe/Warsaw /etc/localtime && \
    apt-get install -y tzdata && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    apt install -y \
 lsb-release sudo vim curl gnupg gpg apt-utils software-properties-common \
 build-essential ninja-build git git-lfs ssh-client jq x11-apps mesa-vulkan-drivers && \
# adding user
    addgroup --gid ${REMOTE_GID} ${REMOTE_USER} && \
    adduser --home /home/k.mrozowski --disabled-password --gecos '' --uid ${REMOTE_UID} --gid ${REMOTE_GID} ${REMOTE_USER} && \
    adduser ${REMOTE_USER} sudo && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

ENV HOME /home/k.mrozowski
USER ${REMOTE_USER}
WORKDIR $HOME

RUN sudo apt update && sudo apt install -y locales && \
    sudo locale-gen en_US en_US.UTF-8 && \
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    echo "export LANG=en_US.UTF-8" >> $HOME/.bashrc

# upgrade clang to 20
RUN sudo apt install -y wget &&\
    wget -qO- https://apt.llvm.org/llvm.sh | sudo bash -s -- 20 &&\
    sudo apt install -y \
 ca-certificates pkg-config gdb libopengl-dev wget libglib2.0-dev libboost-all-dev lldb-20 &&\
    mkdir $HOME/sdk

WORKDIR $HOME/sdk

# install ROS 2 Humble
RUN sudo apt update &&\
    sudo apt install -y software-properties-common &&\
    sudo add-apt-repository universe &&\
    sudo apt update &&\
    sudo apt install -y curl gnupg lsb-release &&\
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg &&\
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null &&\
    sudo apt update &&\
    sudo apt upgrade -y &&\
    sudo rm /bin/sh &&\
    sudo ln -s /usr/bin/bash /bin/sh &&\
    sudo apt-get update -y && \
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration &&\
    sudo apt install -y ros-humble-desktop &&\
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN sudo apt update &&\
    sudo apt install -y python3-catkin-pkg \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    python3-pip \
    python-setuptools &&\
    rm -rf /var/lib/apt/lists/*rm

RUN sudo add-apt-repository --update -y ppa:ubuntu-toolchain-r/test &&\
    wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | sudo apt-key add - &&\
    sudo apt update &&\
    sudo apt install -y \
    bash-completion \
    gdb \
    git-lfs \
    x11-apps \
    libgmp3-dev \
    pybind11-dev

RUN sudo apt update &&\
    sudo apt install -y \
    ros-humble-xacro \
    ros-humble-angles \
    ros-humble-ros2-control \
    ros-humble-realtime-tools \
    ros-humble-control-toolbox \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ament-cmake-clang-format \
    ros-humble-dynamic-edt-3d \
    ros-humble-rviz-visual-tools \
    ros-humble-control-msgs \
    ros-humble-sensor-msgs \
    ros-humble-vision-msgs \
    ros-humble-rosbridge-msgs \
    ros-humble-ros2bag \
    ros-humble-cv-bridge

# upgrade cmake to 3.31.8
RUN wget https://github.com/Kitware/CMake/releases/download/v3.31.8/cmake-3.31.8-linux-x86_64.sh &&\
    chmod +x cmake-3.31.8-linux-x86_64.sh &&\
    sudo yes | ./cmake-3.31.8-linux-x86_64.sh &&\
    sudo cp -r cmake-3.31.8-linux-x86_64/* /usr &&\
    rm -r cmake-3.31.8-linux-x86_64*

# cuda-toolkit
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin &&\
    sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600 &&\
    sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub &&\
    sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/ /" &&\
    wget https://developer.download.nvidia.com/compute/cuda/12.3.0/local_installers/cuda-repo-ubuntu2204-12-3-local_12.3.0-545.23.06-1_amd64.deb &&\
    sudo dpkg -i cuda-repo-ubuntu2204-12-3-local_12.3.0-545.23.06-1_amd64.deb &&\
    rm cuda-repo-ubuntu2204-12-3-local_12.3.0-545.23.06-1_amd64.deb &&\
    sudo cp /var/cuda-repo-ubuntu2204-12-3-local/cuda-*-keyring.gpg /usr/share/keyrings/ &&\
    sudo apt-get update &&\
    sudo apt-get -y install\
 cuda-toolkit-12-3\
 cuda-12-3 \
 ros-humble-rosbag2-storage-mcap\
 libstdc++-13-dev\
 clinfo ocl-icd-opencl-dev\
 libcudnn8-dev\
 libcudnn8\
 cmake-curses-gui\
 g++-12\
 gcc-12\
 python3-dev\
 python3-numpy\
 python3-pip

RUN wget -qO- https://apt.llvm.org/llvm.sh | sudo bash -s -- 20

RUN sudo echo "deb http://archive.ubuntu.com/ubuntu focal main universe"  | sudo tee -a /etc/apt/sources.list &&\
    cat /etc/apt/sources.list &&\
    sudo rm /etc/apt/sources.list.d/* &&\
    sudo apt-get update -q &&\
    sudo apt-get -y install\
 gcc-8\
 g++-8

# gtsam
RUN git clone https://github.com/borglab/gtsam.git &&\
    cd gtsam &&\
    git checkout release/4.2 &&\
    cmake -Bbuild -GNinja -DCMAKE_INSTALL_PREFIX=/usr -DGTSAM_USE_SYSTEM_EIGEN=ON &&\
    cmake --build build &&\
    sudo cmake --build build -t install &&\
    cd - &&\
    rm -rf gtsam

RUN echo "if [ -d \"/usr/local/cuda/bin\" ] ; then PATH=\"/usr/local/cuda/bin:\$PATH\"; fi" >> $HOME/.profile &&\
    echo "export MESA_GL_VERSION_OVERRIDE=3.3" >> $HOME/.bashrc &&\
    echo "export CUDACXX=/usr/local/cuda/bin/nvcc" >> $HOME/.bashrc

# gtsam_points
RUN source $HOME/.profile &&\
    git clone https://github.com/koide3/gtsam_points.git &&\
    cd gtsam_points &&\
    git checkout 09e8c663ff &&\
    cmake -Bbuild -GNinja -DBUILD_WITH_CUDA=ON -DBUILD_WITH_CUDA_MULTIARCH=ON -DCMAKE_INSTALL_PREFIX=/usr &&\
    cmake --build build &&\
    sudo cmake --build build -t install &&\
    cd - &&\
    rm -rf gtsam_points

# Livox-SDK2
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git &&\
    cd Livox-SDK2 &&\
    git checkout v1.2.5 &&\
    cmake -Bbuild -GNinja -DCMAKE_INSTALL_PREFIX=/usr &&\
    cmake --build build &&\
    sudo cmake --build build -t install &&\
    cd - &&\
    rm -rf Livox-SDK2

# iridescence
RUN git clone --recurse-submodules https://github.com/koide3/iridescence.git &&\
    cd iridescence &&\
    git checkout 666d10bbfdf6fed667f88ac21558cbbb1beee209 &&\
    cmake -Bbuild -GNinja -DCMAKE_INSTALL_PREFIX=/usr &&\
    cmake --build build &&\
    sudo cmake --build build -t install &&\
    cd - &&\
    rm -rf iridescence &&\
    cd $HOME && rm -rf sdk

RUN sudo apt update &&\
    sudo apt install -y \
 nlohmann-json3-dev clang-14 libomp-14-dev clangd-14

WORKDIR $HOME

# this value will be overwritten with that one from devcontainer.json
ARG DOMAIN_ID=17

# add environmental variables and aliases
RUN sudo apt update &&\
    sudo apt install -y gcc-9 g++-9 &&\
    sudo sed -i '$ d' /etc/apt/sources.list &&\
    sudo apt update &&\
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 50 &&\
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90 &&\
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 50 &&\
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 90 &&\
    echo "export ROS_DOMAIN_ID=${DOMAIN_ID}" >> $HOME/.bashrc &&\
    CMAKE_ARGUMENTS_COMMON="-DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DTCNN_CUDA_ARCHITECTURES=86 -DCMAKE_CUDA_ARCHITECTURES=86" &&\
    echo -e "alias runColcon=\"colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo $CMAKE_ARGUMENTS_COMMON\"" >> $HOME/.bash_aliases &&\
    echo -e "alias runColconVerbose=\"colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo $CMAKE_ARGUMENTS_COMMON\"" >> $HOME/.bash_aliases &&\
    echo -e "alias runColconPkgs=\"colcon build --cmake-args $CMAKE_ARGUMENTS_COMMON --packages-up-to\"" >> $HOME/.bash_aliases &&\
    echo -e "alias runColconPkgsVerbose=\"colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo $CMAKE_ARGUMENTS_COMMON --packages-up-to\"" >> $HOME/.bash_aliases &&\
    echo -e "alias runColconPkgsDebug=\"colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Debug $CMAKE_ARGUMENTS_COMMON --packages-up-to\"" >> $HOME/.bash_aliases &&\
    mkdir Develop

SHELL ["/bin/bash", "-c"]

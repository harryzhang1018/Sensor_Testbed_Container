FROM nvidia/cuda:12.1.1-devel-ubuntu20.04

ENV LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    DEBIAN_FRONTEND=noninteractive \
    ROS2_WS=/root/dev_ws

WORKDIR $ROS2_WS

RUN apt update && apt upgrade -y && apt install -y git curl gnupg2 lsb-release software-properties-common \
    && add-apt-repository universe \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt update \ 
    && apt install -y ros-foxy-desktop \
    && mkdir src \
    && cd src \
    && git clone https://github.com/ros/ros_tutorials.git -b foxy-devel

RUN apt-get install python3-rosdep -y \
    && rosdep init \
    && rosdep update \
    && rosdep install -i --from-path src --rosdistro foxy -y \
    && apt install python3-colcon-common-extensions -y

# COPY ros2_entrypoint.sh /root/.
# ENTRYPOINT ["/root/ros2_entrypoint.sh"]
# CMD ["bash"]

RUN cd ${ROS2_WS} \
    && . /opt/ros/foxy/setup.sh \
    && colcon build 
    #&& source ${ROS2_WS}/install/setup.bash
# # build source
# RUN colcon \
#     build \
#     --cmake-args \
#       -DSECURITY=ON --no-warn-unused-cli \
#     --symlink-install

# setup bashrc
RUN cp /etc/skel/.bashrc ~/
# setup entrypoint
COPY ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

############ SECTION 2 ------Chrono------- ############
# Chrono dependencies
RUN apt update \
    && apt install -y unzip wget python3 python3-pip git cmake ninja-build doxygen \ 
    libvulkan-dev pkg-config libirrlicht-dev freeglut3-dev mpich libasio-dev \ 
    libboost-dev libglfw3-dev libglm-dev libglew-dev libtinyxml2-dev swig \ 
    python3-dev libhdf5-dev libnvidia-gl-530 \
    && apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

RUN mkdir /root/.ssh/ /opt/optix75 chrono-internal

COPY optix-7.5.0 /opt/optix75
COPY chrono-internal chrono-internal/
RUN mkdir chrono-internal/build \
    && cd chrono-internal/build && cmake ../ -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_BENCHMARKING=OFF \
        -DBUILD_DEMOS=OFF \
        -DBUILD_TESTING=OFF \
        -DENABLE_MODULE_IRRLICHT=ON \
        -DENABLE_MODULE_POSTPROCESS=ON \
        -DENABLE_MODULE_PYTHON=OFF \
        -DENABLE_MODULE_SENSOR=ON \
        -DENABLE_OPENMP=ON \
        -DENABLE_MODULE_VEHICLE=ON \
        -DEigen3_DIR=/usr/lib/cmake/eigen3 \
        -DOptiX_INCLUDE=/opt/optix75/include \
        -DOptiX_INSTALL_DIR=/opt/optix75 \
        -DUSE_CUDA_NVRTC=OFF \
        -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs \
        -DCUDA_NVCC_EXECUTABLE=/usr/local/cuda/bin/nvcc \
        -DCUDA_ARCH_NAME=Turing \ 
        && ninja && ninja install

RUN mkdir -p chrono-internal/chrono_sensor_ros_node/build
ADD chrono_sensor_ros_node /root/dev_ws/chrono-internal/chrono_sensor_ros_node
RUN cd chrono-internal/chrono_sensor_ros_node/build && cmake ../ -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DChrono_DIR=/root/dev_ws/chrono-internal/build/cmake/ \
    && ninja

RUN mkdir src/image_subscriber
COPY streamer.py /root/dev_ws/src/image_subscriber/

RUN apt update && apt install -y tmux 
#xfce4 xfce4-goodies tightvncserver
FROM nvidia/cuda:11.4.0-devel-ubuntu20.04

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN apt update && apt install -y git curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-foxy-desktop


WORKDIR /root/dev_ws/src
RUN git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
WORKDIR /root/dev_ws

RUN apt-get install python3-rosdep -y
RUN rosdep init
RUN rosdep update
RUN rosdep install -i --from-path src --rosdistro foxy -y
RUN apt install python3-colcon-common-extensions -y


# COPY ros2_entrypoint.sh /root/.
# ENTRYPOINT ["/root/ros2_entrypoint.sh"]
# CMD ["bash"]

# clone source
ENV ROS2_WS /root/dev_ws
WORKDIR $ROS2_WS


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
COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

############ SECTION 2 ------Chrono------- ############
#chrono dependency installed here
RUN apt update && apt install -y libirrlicht-dev libnvidia-gl-515 libeigen3-dev cmake cmake-curses-gui libglu1-mesa-dev freeglut3-dev mesa-common-dev wget swig libglfw3 libglfw3-dev x11proto-gl-dev glew-utils git libxxf86vm-dev libglew-dev openmpi-common libopenmpi-dev ninja-build python3-numpy

# Clean up to reduce image size
RUN apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

#RUN wget https://uwmadison.box.com/shared/static/97fkm979iuccls990ottx5g5bpva8pwe.sh -O optix75.sh
COPY NVIDIA-OptiX-SDK-7.5.0-linux64-x86_64.sh ./optix75.sh
RUN chmod +x optix75.sh
RUN mkdir /opt/optix75
RUN ./optix75.sh --prefix=/opt/optix75 --skip-license
RUN rm optix75.sh
RUN git clone https://github.com/projectchrono/chrono.git -b feature/sensor
RUN mkdir chrono/build
RUN cd chrono/build && cmake ../ -G Ninja \
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
 #-DNUMPY_INCLUDE_DIR=/usr/lib/python3/dist-packages/numpy/core/include \
 && ninja && sudo ninja install



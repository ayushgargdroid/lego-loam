FROM osrf/ros:melodic-desktop-full

LABEL maintainer "garg.ayu@northeastern.edu"

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN sudo apt-get -y update && sudo apt-get install -y git wget
RUN /bin/bash -c "curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py"
RUN /bin/bash -c "python get-pip.py"
RUN sudo apt-get install libpcl-*1.8
RUN sudo apt-get update 
RUN sudo apt-get install -yqq  build-essential ninja-build \
    python3-dev python3-pip apt-utils curl git cmake unzip autoconf autogen \
    libtool mlocate zlib1g-dev python3-numpy python3-wheel wget \
    software-properties-common openjdk-8-jdk libpng-dev  \
    libxft-dev ffmpeg python3-pyqt5.qtopengl ros-melodic-tf2-sensor-msgs
RUN sudo apt-get install -yqq  build-essential python3-dev python3-pip \
    apt-utils git cmake libboost-all-dev libyaml-cpp-dev libopencv-dev \ 
    python-empy
RUN sudo pip install catkin_tools trollius numpy

# Make catkin_ws
RUN /bin/bash -c "mkdir ~/slam && mkdir ~/slam/slam_ws && mkdir ~/slam/slam_ws/src"
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && cd ~/slam/slam_ws && catkin_make"
RUN /bin/bash -c "echo 'source /opt/ros/melodic/setup.bash' >> ~/.bashrc"
RUN /bin/bash -c "echo 'source ~/slam/slam_ws/devel/setup.bash' >> ~/.bashrc"
RUN /bin/bash -c "source ~/.bashrc"


# Installing Geographic Lib
RUN /bin/bash -c "cd ~/slam/ && wget https://github.com/ayushgargdroid/lego-loam/raw/gps_base/GeographicLib-1.50.1.tar.gz"
RUN /bin/bash -c "cd ~/slam/ && tar -xf GeographicLib-1.50.1.tar.gz"
RUN /bin/bash -c "cd ~/slam/GeographicLib-1.50.1/ && mkdir build" 
RUN /bin/bash -c "cd ~/slam/GeographicLib-1.50.1/build/ && cmake .. && sudo make install -j8" 

# Installing GtSam
RUN /bin/bash -c "wget -O ~/slam/gtsam.zip https://github.com/borglab/gtsam/archive/4.0.0-alpha2.zip"
RUN /bin/bash -c "cd ~/slam/ && unzip gtsam.zip -d ~/slam/"
RUN /bin/bash -c "cd ~/slam/gtsam-4.0.0-alpha2/ && mkdir build" 
RUN /bin/bash -c "cd ~/slam/gtsam-4.0.0-alpha2/build/ && cmake -D GTSAM_WITH_EIGEN_MKL=OFF -D CMAKE_CXX_FLAGS="-w" .. && sudo make install -j8" 

# Start building Lego Loam
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && cd ~/slam/slam_ws && catkin_make"
RUN /bin/bash -c "cd ~/slam/slam_ws/src && git clone https://github.com/ayushgargdroid/lego-loam.git"
RUN /bin/bash -c "cd ~/slam/slam_ws/src && git clone https://github.com/ayushgargdroid/imu_transformer.git"
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash && cd ~/slam/slam_ws && catkin_make"
CMD /bin/bash -c "source /opt/ros/melodic/setup.bash && cd ~/slam/slam_ws && source devel/setup.bash && roslaunch lego_loam run.launch"








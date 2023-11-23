# Start from a C++ base image with necessary compilers
FROM ubuntu:latest

RUN rm -rf CMakeCache.txt && rm -rf CMakeFiles

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive


# Add the repository for GCC-9
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository ppa:ubuntu-toolchain-r/test

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    gcc-9 \
    g++-9 \
    cmake \
    git \
    ccache \
    libeigen3-dev \
    libglew-dev \
    libjpeg-dev \
    libpng-dev \
    liblz4-dev \
    libbz2-dev \
    libboost-regex-dev \
    libboost-filesystem-dev \
    libboost-date-time-dev \
    libboost-program-options-dev \
    libgtest-dev \
    libopencv-dev
    



    
# Update alternatives to use GCC-9 and G++-9
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 100 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-9 100
    

#RUN apt-get install -y libtbb-dev=2020.1-2 libtbb2=2020.1-2
#RUN apt-get install -y libtbb-dev libtbb2
#RUN ls -l /usr/include/tbb
#RUN find /usr -name "tbb_stddef.h"


#ENV TBB_ROOT /usr
#RUN ls /usr/include/tbb
#RUN apt list --installed | grep libtbb

#ENV TBB_ROOT /usr
#ENV TBB_INCLUDE_DIR /usr/include
#ENV TBB_LIBRARY_DIR /usr/lib/x86_64-linux-gnu
    
# Reset DEBIAN_FRONTEND
ENV DEBIAN_FRONTEND=dialog

# Clone the Basalt SLAM repository
#RUN git clone https://github.com/VladyslavUsenko/basalt.git

# Set the working directory to the cloned repository
WORKDIR /project
COPY . /project

# If there are any specific submodules or branches needed, fetch them
# RUN git submodule update --init --recursive

# Build the project
RUN mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo &&\
    make -j$(nproc)

# Set the default command to execute
# Replace this with the command to run the SLAM system
CMD ["./build/granite_vio", "--dataset-path", "/data/frames", "--cam-calib", "/data/calib_tii.json", "--dataset-type", "euroc", "--config-path", "/data/config_tii.json", "--use-imu", "0", "--show-gui", "1", "--step-by-step", "0"]




FROM ubuntu:16.04

RUN apt-get update && apt-get install -y cmake git libtbb-dev libeigen3-dev libglew-dev ccache libjpeg-dev libpng12-dev openssh-client liblz4-dev libbz2-dev libboost-regex-dev libboost-filesystem-dev libboost-date-time-dev libboost-program-options-dev libopencv-dev libpython2.7-dev wget libgtest-dev software-properties-common

RUN add-apt-repository ppa:ubuntu-toolchain-r/test

RUN apt update && apt-get install -y g++-7


RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 60 \
                         --slave /usr/bin/g++ g++ /usr/bin/g++-7
RUN gcc --version
RUN g++ --version

RUN wget https://cmake.org/files/v3.13/cmake-3.13.4-Linux-x86_64.sh
RUN chmod +x cmake-3.13.4-Linux-x86_64.sh
RUN ./cmake-3.13.4-Linux-x86_64.sh  --skip-license --prefix=/usr/local
RUN update-alternatives --install /usr/bin/cmake cmake /usr/local/bin/cmake 1 --force
RUN cmake --version
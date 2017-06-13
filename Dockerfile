FROM ubuntu:xenial

RUN apt-get update && apt-get install -y \
    build-essential \
    sudo \
    gcc \
    g++ \
    gfortran \
    cmake \
    pkg-config \
    unzip \
    git \
    wget \
    cppad \
    python-matplotlib \ 
    python2.7-dev
    
RUN apt-get install -y \
    openssl libssl-dev zlib1g

ADD install-docker.sh .
RUN bash install-docker.sh

# go to our home dir and copy contents of our host dir...
WORKDIR /home
COPY . /home
COPY CMakeLists.txt CMakeLists.txt
RUN cmake .
RUN make

EXPOSE 4567

# Use the official Ubuntu 22.04 base image
FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# Install dependencies
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    wget \
    nano \
    libceres-dev \
    libgl1-mesa-glx \
    libglu1-mesa \
    mesa-utils \
    libx11-dev \
    libxrender-dev \
    libxtst-dev

CMD ["/bin/bash"]

# Create new user
RUN useradd -ms /bin/bash dev \
    && echo "dev:525407" | chpasswd && adduser dev sudo \
    && usermod -aG sudo dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to new user
USER dev

# Switch main workspace to $HOME of user
WORKDIR /home/dev

# Start doing things below!


# How to Devcontainers

## Directory setup

Within your main folder, I'll just call it `workspace`, create a `.devcontainer` folder and put two files:
- Dockerfile
- devcontainer.json

Also create a folder for persistent storage, I'll call it `pocket`

```
|-- workspace/
|   |-- .devcontainer/
|   |   |-- .devcontainer.json
|   |   |-- Dockerfile
|   |-- pocket/
|   |   |-- HowTo.md (this!)
```

## Setup Dockerfile

You can just put whatever, here is what I have as a base

```Dockerfile
# Use the official Ubuntu 22.04 base image
FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    nano \
    libceres-dev \
    libgl1-mesa-glx \
    libglu1-mesa \
    mesa-utils \
    libx11-dev \
    libxrender-dev \
    libxtst-dev

CMD ["/bin/bash"]

# Create new user (change name and password if you want)
RUN useradd -ms /bin/bash dev \
    && echo "dev:nickreed" | chpasswd \
    && usermod -aG sudo dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to new user
USER dev

# Switch main workspace to $HOME of user
WORKDIR /home/dev

# Start doing things below!

```

## Setup `devcontianer.json`

Here is what I have

```json
{
    "name": "Example", //CHANGE ME
    // "image": "omnipotence:noetic",
    "build": {
        "dockerfile": "Dockerfile", // Path to your Dockerfilez
        "options": [
            "-t=my_image:my_tag" //CHANGE ME
        ]
    },
    "workspaceFolder": "/home/dev",
    "remoteUser": "dev",
    "mounts": [
        "source=${localWorkspaceFolder}/config,target=/home/dev/pocket,type=bind,consistency=cached"
    ],
    "runArgs": [
        "--env=DISPLAY=${localEnv:DISPLAY}",
        "--env", "LIBGL_ALWAYS_INDIRECT=0",
        "--volume", "/tmp/.X11-unix:/tmp/.X11-unix",
        "--volume", "/dev/dri:/dev/dri",
        "--network=host",
        "--hostname", "container",
        "--name", "my_container", //CHANGE ME
        "--rm"
    ]
}
```
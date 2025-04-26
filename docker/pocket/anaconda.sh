#!/bin/bash -i

install_anaconda(){
    wget https://repo.anaconda.com/archive/Anaconda3-2024.10-1-Linux-x86_64.sh -O anaconda_installer.sh
    bash anaconda_installer.sh -b
    source ~/.bashrc
    rm anaconda_installer.sh

    source ~/anaconda3/bin/activate
    conda init --all

    source ~/.bashrc
} 

install_anaconda
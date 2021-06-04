FROM ubuntu:20.04

# Define as informações do container
LABEL maintainer="RoboFEI" \
      name="prometheus_controller" \
      version="1.0"

# Define variáveis do OS (export)
ENV DEBBIAN_FRONTEND=noninteractive \
    ROBOCUP_ROBOT_ID=1\
    ROBOCUP_TEAM_COLOR=red \
    ROBOCUP_SIMULATOR=192.168.123.22 \
    ROBOCUP_SIMULATOR_ADDR=$ROBOCUP_SIMULATOR:1000$ROBOCUP_ROBOT_ID \
    ROBOCUP_MIRROR_SERVER_IP=3737\
    ROBOCUP_GAMECONTROLLER_IP=3939 \
    CONTROLLER=/RoboCup/kidsize/controllers/player

# Instalação de packages
RUN apt update -y \
    && apt upgrade -y \
    && apt-get install -y \
        git \
    && apt-get clean autoclean \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/list/*

# Obtendo o controlador do github e variáveis de conexão
RUN mkdir -p /RoboCup \
    && git clone https://github.com/RoboFEI/RoboFEI-2021.git /RoboCup

# Controlador port
EXPOSE 3939

# Indo até o diretório do controlador
WORKDIR $CONTROLLER
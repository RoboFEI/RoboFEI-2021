FROM ubuntu:20.04

# Define as informações do container
LABEL maintainer="RoboFEI" \
      name="prometheus_controller" \
      version="1.0"

# Define variáveis do OS (export)
ENV DEBBIAN_FRONTEND=noninteractive \
    ROBOCUP_SIMULATOR=192.168.1.100 \
    ROBOCUP_ROBOT_ID=10021 \
    ROBOCUP_TEAM_COLOR=blue \
    ROBOCUP_SIMULATOR_ADDR=$ROBOCUP_SIMULATOR:$ROBOCUP_ROBOT_ID\
    CONTROLLER=/RoboCup/kidsize/controllers/Control

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
EXPOSE 3838

# Indo até o diretório do controlador
WORKDIR $CONTROLLER
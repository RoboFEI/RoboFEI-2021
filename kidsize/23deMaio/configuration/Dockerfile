FROM ubuntu:20.04

# Define as informações do container
LABEL maintainer="RoboFEI" \
      name="prometheus_controller" \
      version="1.0"

# Define variáveis do OS (export)
ENV DEBBIAN_FRONTEND noninteractive
ENV ROBOCUP_SIMULATOR 192.168.1.100
ENV ROBOCUP_ROBOT_ID 10021
ENV ROBOCUP_TEAM_COLOR blue
ENV ROBOCUP_SIMULATOR_ADDR $ROBOCUP_SIMULATOR:$ROBOCUP_ROBOT_ID

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
    && git clone https://github.com/KValim/ROBO-FEI.git /RoboCup

# Controlador port
EXPOSE 3838
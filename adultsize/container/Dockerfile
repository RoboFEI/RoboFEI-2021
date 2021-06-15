FROM ubuntu:20.04

# Define as informações do container
LABEL maintainer="RoboFEI" \
      name="cronus_controller" \
      version="1.1"

# Define variáveis do sistema (export)
ENV DEBIAN_FRONTEND=noninteractive \
    CONTROLLER=/RoboCup/adultsize/controllers/player

# Instalação de packages

RUN apt-get update -y
RUN apt-get upgrade -y 
RUN apt-get install -y build-essential 
RUN apt-get install -y git 
RUN apt-get autoremove -y

# Obtendo o controlador do github e variáveis de conexão
RUN mkdir -p /RoboCup \
    && cd RoboCup \
    && git clone https://github.com/RoboFEI/RoboFEI-2021.git /RoboCup


RUN apt-get install -y wget
RUN wget http://archive.ubuntu.com/ubuntu/pool/main/p/protobuf/libprotobuf17_3.6.1.3-2ubuntu5_amd64.deb \
    && dpkg -i libprotobuf17_3.6.1.3-2ubuntu5_amd64.deb 

# Install sudoers file
ADD sudoers /etc/sudoers

# Indo até o diretorio do controlador
WORKDIR $CONTROLLER

# Setup runtime
ENV DEBIAN_FRONTEND=readline
COPY entrypoint.sh /usr/local/bin/entrypoint

ENTRYPOINT ["sh","/usr/local/bin/entrypoint"]
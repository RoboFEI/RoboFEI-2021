FROM ubuntu:20.04

# Define as informações do container
LABEL maintainer="RoboFEI" \
      name="prometheus_controller" \
      version="1.0"

# Define variáveis do sistema (export)
ENV DEBIAN_FRONTEND=noninteractive \
    CONTROLLER=/RoboCup/kidsize/controllers/player

# Instalação de packages
RUN apt update -y && apt-get upgrade -y 
RUN apt install -y build-essential 
RUN apt install -y python-dev
RUN apt install -y python-is-python3
RUN apt install -y python3-pip
RUN apt install -y git 
RUN apt install -y libzmq3-dev
RUN apt autoremove -y

# Python dependencies
RUN pip install zmq
RUN pip install construct



# Obtendo o controlador do github e variáveis de conexão
RUN mkdir -p /RoboCup && cd RoboCup \
    && git clone https://github.com/RoboFEI/RoboFEI-2021.git /RoboCup


RUN apt-get install -y wget
RUN wget http://archive.ubuntu.com/ubuntu/pool/main/p/protobuf/libprotobuf17_3.6.1.3-2ubuntu5_amd64.deb \
    && dpkg -i libprotobuf17_3.6.1.3-2ubuntu5_amd64.deb 

# Install sudoers file
ADD sudoers /etc/sudoers

# Indo até o diretorio do controlador
WORKDIR $CONTROLLER

# Setup runtime
ENV DEBIAN_FRONTEND=readline \
    ROBOCUP_MIRROR_SERVER_IP=kvalim 
#####################################tirar linha de cima

COPY entrypoint.sh /usr/local/bin/entrypoint

EXPOSE 3737

ENTRYPOINT ["sh","/usr/local/bin/entrypoint"]
FROM ubuntu:20.04

# Define as informações do container
LABEL maintainer="RoboFEI" \
      name="prometheus_controler" \
      version="1.0"

# Define variáveis do OS
ENV DEBBIAN_FRONTEND noninteractive

# Instalação de packages
RUN apt update -y \
    && apt upgrade -y \
    && apt-get install -y \
        git \
    && apt-get clean autoclean \
    && apt-get autoremove -y \
    && rm -rf /var/lib/apt/list/*

# Adicionando o script de CMD
#ADD start.sh /start.sh

# Obtendo o controlador do github e atribuindo prioridade ao .sh
RUN mkdir -p /RoboCup \
    && git clone https://github.com/KValim/ROBO-FEI.git /RoboCup
#RUN chmod +x /start.sh

# Controlador port
EXPOSE 3838

# Rodando o script
#CMD ["/start.sh"]
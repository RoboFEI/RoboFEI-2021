#!/usr/bin/env bash


cd /RoboCup/kidsize/controllers/player/Communication
python receiver.py


gnome-terminal cd /RoboCup/kidsize/controllers/player
ADDR=${ROBOCUP_SIMULATOR_ADDR:=kvalim:10001}

IP=$(echo $ADDR | cut -d: -f1)
PORT=$(echo $ADDR | cut -d: -f2)
ROBOTID=${ROBOCUP_ROBOT_ID:=1}
TEAMMIRRORIP=${IP}
TEAMMIRRORPORT=3737
ARGS=${FUMANOIDS_ARGS}
BIN=${FUMANOIDS_BIN:=/RoboCup/kidsize/controllers/player/client}

while true; do
  ${BIN} --webots --webots.address "$IP" \
           --webots.port "$PORT" --robotid "${ROBOTID}" \
           --referee.ip "${IP}" --teammirror.ip "${TEAMMIRRORIP}" --teammirror.port "${TEAMMIRRORPORT}" \
           ${ARGS}

  sleep 1
done
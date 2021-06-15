// Copyright 1996-2021 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <string.h>
#include <iostream>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include "messages.pb.h"
#if GOOGLE_PROTOBUF_VERSION < 3006001
#define ByteSizeLong ByteSize
#endif

#include "robot_client.hpp"


static void usage(const std::string &error_msg = "") {
  if (error_msg.length() > 0)
    fprintf(stderr, "Invalid call: %s\n", error_msg.c_str());
  fprintf(stderr, "Usage: client [-v verbosity_level] <host> <port>\n");
}

int flag = 0;
int flag_penalty = 0;
int flag_time = 0;
int flag_time_penalty = 0;
int stop_walk = 0;

void tchau(RobotClient client){
  const char *move;
  const char *levanta_braco = "motor_positions {name: 'LeftArmPitch [arm]'  position: -2.434}  motor_positions {name: 'LeftShoulderRoll [shoulder]'  position: 2}";
  const char *levanta = "motor_positions {  name: 'LeftElbowPitch [arm]'  position: 2.1}";
  const char *abaixa = "motor_positions {  name: 'LeftElbowPitch [arm]'  position: 1.2}";

  SensorMeasurements sensors = client.receive();
  int time = sensors.time();

  if(stop_walk<5)
  {
    if(flag == 0) flag_time = time;
    flag++;

    if(time<(500+flag_time)) move = levanta_braco;
    else if(time <(1000+flag_time)) move = levanta;
    else
    {
      move = abaixa;
      flag = 0;
      //stop_wave++;
    }
  }

  ActuatorRequests request = RobotClient::buildRequestMessage(move);
  client.sendRequest(request);

  //std::string out = google::protobuf::TextFormat::PrintToString(sensors.position_sensors(), &out);
  //std::cout << sensors.position_sensors(1).value() << std::endl;
  //std::string printout;
  //google::protobuf::TextFormat::PrintToString(sensors, &printout);
  //std::cout << time << std::endl;
  //std::cout << motorPosition.position() << std::endl;
  
 

}

void walk(RobotClient client){
  const char *move;
  const char *phase1 = "./walk/1.txt"; //"walk_ready"
  const char *phase2 = "./walk/2.txt"; //levanta pe direito
  const char *phase3 = "./walk/3.txt";
  const char *phase4 = "./walk/4.txt";
  const char *phase5 = "./walk/5.txt"; //"walk_ready'"
  const char *phase6 = "./walk/6.txt"; //levanta pe esquerdo
  const char *phase7 = "./walk/7.txt";
  const char *phase8 = "./walk/8.txt";


  SensorMeasurements sensors = client.receive();
  int time = sensors.time();

  if(stop_walk<5)
  {
    if(flag == 0) flag_time = time;
    flag++;

    if(time<(50+flag_time)) move = phase1;
    else if(time <(100+flag_time)) move = phase2;
    else if(time <(150+flag_time)) move = phase3;
    else if(time <(200+flag_time)) move = phase4;
    else if(time <(250+flag_time)) move = phase5;
    else if(time <(300+flag_time)) move = phase6;
    else if(time <(350+flag_time)) move = phase7;
    else
    {
      move = phase8;
      flag = 0;
      //stop_wave++;
    }
  }

  ActuatorRequests request = RobotClient::buildRequestMessage(move);
  client.sendRequest(request);

}

void kick_right_weak(RobotClient client){
  const char *move;
  const char *phase1 = "./kick_right_weak/1.txt"; //abre os bracos
  const char *phase2 = "./kick_right_weak/2.txt"; //fecha os bracos
  const char *phase3 = "./kick_right_weak/3.txt"; //pe para tras
  const char *phase4 = "./kick_right_weak/4.txt"; //chuta
  const char *phase5 = "./kick_right_weak/5.txt"; //retorna o pe
  const char *phase6 = "./kick_right_weak/6.txt"; //abre os bracos

  SensorMeasurements sensors = client.receive();
  int time = sensors.time();

  if(stop_walk<5)
  {
    if(flag == 0) flag_time = time;
    flag++;

    if(time<(500+flag_time)) move = phase1;
    else if(time <(1000+flag_time)) move = phase2;
    else if(time <(1500+flag_time)) move = phase3;
    else if(time <(2000+flag_time)) move = phase4;
    else if(time <(2500+flag_time)) move = phase5;
    else
    {
      move = phase6;
      flag = 0;
      //stop_walk++;
    }
  }

  ActuatorRequests request = RobotClient::buildRequestMessage(move);
  client.sendRequest(request);
}

void penalty(RobotClient client){
  const char *move;

  SensorMeasurements sensors = client.receive();
  int time = sensors.time();

  if(stop_walk<=38)
  {
    const char *phase1 = "./walk/1.txt"; //"walk_ready"
    const char *phase2 = "./walk/2.txt"; //levanta pe direito
    const char *phase3 = "./walk/3.txt";
    const char *phase4 = "./walk/4.txt";
    const char *phase5 = "./walk/5.txt"; //"walk_ready'"
    const char *phase6 = "./walk/6.txt"; //levanta pe esquerdo
    const char *phase7 = "./walk/7.txt";
    const char *phase8 = "./walk/8.txt";

    if(flag == 0) flag_time = time;

    flag++;

    if(time<(50+flag_time)) move = phase1;
    else if(time <(100+flag_time)) move = phase2;
    else if(time <(150+flag_time)) move = phase3;
    else if(time <(200+flag_time)) move = phase4;
    else if(time <(250+flag_time)) move = phase5;
    else if(time <(300+flag_time)) move = phase6;
    //else if(time <(350+flag_time)) move = phase7;
    else
    {
      move = phase7;
      flag = 0;
      stop_walk++;
    }
  }

  if(stop_walk > 38)
  {
    const char *phase1 = "./kick_right_weak/1.txt"; //abre os bracos

    const char *phase2 = "./kick_right_weak/2.txt"; //pe para tras
    const char *phase3 = "./kick_right_weak/3.txt"; //chuta
    const char *phase4 = "./kick_right_weak/4.txt"; //chuta
    const char *phase5 = "./kick_right_weak/5.txt"; //retorna o pe
    const char *phase6 = "./kick_right_weak/6.txt"; 

    if(flag == 0) flag_time = time;
    flag++;

    if(time<(250+flag_time)) move = phase2;
    else if(time <(400+flag_time)) move = phase3;
    else if(time <(550+flag_time)) move = phase4;
    else if(time <(800+flag_time)) move = phase5;
    //else if(time <(1250+flag_time)) move = phase4;
    //else if(time <(1600+flag_time)) move = phase5;
    else
    {
      move = phase6;
      flag = 0;
      stop_walk = 0;
    }
  }

  ActuatorRequests request = RobotClient::buildRequestMessage(move);
  client.sendRequest(request);
}

/* void penalty(RobotClient client){
  SensorMeasurements sensors = client.receive();
  int time_penalty = sensors.time();

  if(stop_wave<5)
  {
    if(flag_penalty == 0) flag_time_penalty = time_penalty;
    flag_penalty++;

    if(time_penalty<(50+flag_time_penalty)) walk(client);
    else
    {
      kick_right_weak(client);
      flag_penalty = 0;
      //stop_wave++;
    }
  }
} */

int main(int argc, char *argv[]) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  
  int port = -1;
  std::string host;
  int arg_idx = 1;
  int verbosity = 3;
  while (arg_idx < argc) {
    std::string current_arg(argv[arg_idx]);
    // Treating arguments
    if (current_arg[0] == '-') {
      if (current_arg == "-v") {
        if (arg_idx + 1 >= argc)
          usage("Missing value for verbosity");
        verbosity = std::stoi(argv[arg_idx + 1]);
        arg_idx++;
      } else if (current_arg == "-h" || current_arg == "--help")
        usage();
      else
        usage();
    } else if (host.length() == 0)
      host = current_arg;
    else if (port == -1) {
      port = std::stoi(current_arg);
      if (port < 0)
        usage("Unexpected negative value for port: " + current_arg);
    } else
      usage("Unexpected additional argument: " + current_arg);
    arg_idx++;
  }
  if (port == -1)
    usage("Missing arguments");

  RobotClient client(host, port, verbosity);
  client.connectClient();
  while (client.isOk()) {
    try {
      //tchau(client);
      //walk(client);
      //kick_right_weak(client);
      penalty(client);
      
      SensorMeasurements sensors = client.receive();
      std::string printout;
      google::protobuf::TextFormat::PrintToString(sensors, &printout);
    } catch (const std::runtime_error &exc) {
      std::cerr << "Runtime error: " << exc.what() << std::endl;
    }
  }

  return 0;
}

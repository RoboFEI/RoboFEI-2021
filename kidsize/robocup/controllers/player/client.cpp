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


#include <string.h>
#include <iostream>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include "messages.pb.h"
#if GOOGLE_PROTOBUF_VERSION < 3006001
#define ByteSizeLong ByteSize
#endif

#include "robot_client.hpp"

//  Hello World server
#include <zmq.hpp>


#include <unistd.h>
#include <assert.h>


#include <sys/types.h>
#include <sys/stat.h>


#include <fstream>
#include <fcntl.h>
#include <google/protobuf/io/coded_stream.h>
using namespace std;
using google::protobuf::io::FileInputStream;
using google::protobuf::io::FileOutputStream;


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
int flag_caido = 0;

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

  if(flag == 0) flag_time = time;
  flag++;

    if(time<(30+flag_time)) move = phase1;
    else if(time <(60+flag_time)) move = phase2;
    else if(time <(90+flag_time)) move = phase3;
    else if(time <(120+flag_time)) move = phase4;
    else if(time <(150+flag_time)) move = phase5;
    else if(time <(180+flag_time)) move = phase6;
    else if(time <(210+flag_time)) move = phase7;
  else
  {
    move = phase8;
    flag = 0;
  }
  
  ActuatorRequests request = RobotClient::buildRequestMessage(move);
  client.sendRequest(request);
}

void kick_right_weak(RobotClient client){
  const char *move;
  const char *phase1 = "./kick_right_weak/1.txt"; //
  const char *phase2 = "./kick_right_weak/2.txt"; //
  const char *phase3 = "./kick_right_weak/3.txt"; //
  const char *phase4 = "./kick_right_weak/4.txt"; //
  const char *phase5 = "./kick_right_weak/5.txt"; //
  const char *phase6 = "./kick_right_weak/6.txt"; //

  SensorMeasurements sensors = client.receive();
  int time = sensors.time();

  if(flag == 0) flag_time = time;
  flag++;

  if(time<(250+flag_time)) move = phase2;
  else if(time <(400+flag_time)) move = phase3;
  else if(time <(550+flag_time)) move = phase4;
  else if(time <(800+flag_time)) move = phase5;
  else
  {
    move = phase6;
    flag = 0;
    stop_walk = 0;
  }
  

  ActuatorRequests request = RobotClient::buildRequestMessage(move);
  client.sendRequest(request);
}

void kick_left_weak(RobotClient client){
  const char *move;
  const char *phase1 = "./kick_left_weak/1.txt"; //abre os bracos
  const char *phase2 = "./kick_left_weak/2.txt"; //fecha os bracos
  const char *phase3 = "./kick_left_weak/3.txt"; //pe para tras
  const char *phase4 = "./kick_left_weak/4.txt"; //chuta
  const char *phase5 = "./kick_left_weak/5.txt"; //retorna o pe
  const char *phase6 = "./kick_left_weak/6.txt"; //abre os bracos

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

void turn_left(RobotClient client){
  const char *move;
  const char *phase1 = "./turn_left/1.txt"; //
  const char *phase2 = "./turn_left/2.txt"; //
  const char *phase3 = "./turn_left/3.txt"; //
  const char *phase4 = "./turn_left/4.txt"; //
  const char *phase5 = "./turn_left/5.txt"; //
  const char *phase6 = "./turn_left/6.txt"; //
  const char *phase7 = "./turn_left/7.txt"; //
  const char *phase8 = "./turn_left/8.txt"; //

  SensorMeasurements sensors = client.receive();
  int time = sensors.time();

  if(stop_walk<5)
  {
    if(flag == 0) flag_time = time;
    flag++;

    if(time<(100+flag_time)) move = phase1;
    else if(time <(200+flag_time)) move = phase2;
    else if(time <(300+flag_time)) move = phase3;
    else if(time <(400+flag_time)) move = phase4;
    else if(time <(500+flag_time)) move = phase5;
    else if(time <(600+flag_time)) move = phase6;
    else if(time <(700+flag_time)) move = phase7;
    else
    {
      move = phase8;
      flag = 0;
      //stop_walk++;
    }
  }

  ActuatorRequests request = RobotClient::buildRequestMessage(move);
  client.sendRequest(request);
}

void turn_right(RobotClient client){
  const char *move;
  const char *phase1 = "./turn_right/1.txt"; //
  const char *phase2 = "./turn_right/2.txt"; //
  const char *phase3 = "./turn_right/3.txt"; //
  const char *phase4 = "./turn_right/4.txt"; //
  const char *phase5 = "./turn_right/5.txt"; //
  const char *phase6 = "./turn_right/6.txt"; //
  const char *phase7 = "./turn_right/7.txt"; //
  const char *phase8 = "./turn_right/8.txt"; //

  SensorMeasurements sensors = client.receive();
  int time = sensors.time();

  if(stop_walk<5)
  {
    if(flag == 0) flag_time = time;
    flag++;

    if(time<(100+flag_time)) move = phase1;
    else if(time <(200+flag_time)) move = phase2;
    else if(time <(300+flag_time)) move = phase3;
    else if(time <(400+flag_time)) move = phase4;
    else if(time <(500+flag_time)) move = phase5;
    else if(time <(600+flag_time)) move = phase6;
    else if(time <(700+flag_time)) move = phase7;
    else
    {
      move = phase8;
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

  if(stop_walk<=15)
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

    if(time<(30+flag_time)) move = phase1;
    else if(time <(60+flag_time)) move = phase2;
    else if(time <(90+flag_time)) move = phase3;
    else if(time <(120+flag_time)) move = phase4;
    else if(time <(150+flag_time)) move = phase5;
    else if(time <(180+flag_time)) move = phase6;
    else if(time <(210+flag_time)) move = phase7;
    else
    {
      move = phase8;
      flag = 0;
      stop_walk++;
      cout << stop_walk << endl;
    }
  }


  if(stop_walk > 15)
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

void fall_left(RobotClient client){
  const char *move;
  const char *phase1 = "./fall_left/1.txt"; //
  const char *phase2 = "./fall_left/2.txt"; //


  SensorMeasurements sensors = client.receive();
  int time = sensors.time();

  if(stop_walk<5)
  {
    if(flag == 0) flag_time = time;
    flag++;

    if(time<(500+flag_time)) move = phase1;
    else
    {
      move = phase2;
      //flag = 0;
      //stop_walk++;
    }
  }

  ActuatorRequests request = RobotClient::buildRequestMessage(move);
  client.sendRequest(request);
}

void fall_right(RobotClient client){
  const char *move;
  const char *phase1 = "./fall_right/1.txt"; //
  const char *phase2 = "./fall_right/2.txt"; //


  SensorMeasurements sensors = client.receive();
  int time = sensors.time();

  if(stop_walk<5)
  {
    if(flag == 0) flag_time = time;
    flag++;

    if(time<(500+flag_time)) move = phase1;
    else
    {
      move = phase2;
      //flag = 0;
      //stop_walk++;
    }
  }

  ActuatorRequests request = RobotClient::buildRequestMessage(move);
  client.sendRequest(request);
}

void standup_front(RobotClient client){
  const char *move;
  const char *phase1 = "./standup_front/1.txt"; //
  const char *phase2 = "./standup_front/2.txt"; //
  const char *phase3 = "./standup_front/3.txt"; //
  const char *phase4 = "./standup_front/4.txt"; //
  const char *phase5 = "./standup_front/5.txt"; //
  const char *phase6 = "./standup_front/6.txt"; //

  SensorMeasurements sensors = client.receive();
  int time = sensors.time();


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
    flag_caido = 0;
    sleep(1);
  }

  ActuatorRequests request = RobotClient::buildRequestMessage(move);
  client.sendRequest(request);
}

void standup_back(RobotClient client){
  const char *move;
  const char *phase1 = "./standup_back/1.txt"; //
  const char *phase2 = "./standup_back/2.txt"; //
  const char *phase3 = "./standup_back/3.txt"; //
  const char *phase4 = "./standup_back/4.txt"; //
  const char *phase5 = "./standup_back/5.txt"; //
  const char *phase6 = "./standup_back/6.txt"; //
  const char *phase7 = "./standup_back/7.txt"; //

  SensorMeasurements sensors = client.receive();
  int time = sensors.time();


  if(flag == 0) flag_time = time;
  flag++;

  if(time<(500+flag_time)) move = phase1;
  else if(time <(1000+flag_time)) move = phase2;
  else if(time <(1500+flag_time)) move = phase3;
  else if(time <(2000+flag_time)) move = phase4;
  else if(time <(2500+flag_time)) move = phase5;
  else if(time <(3000+flag_time)) move = phase6;
  else
  {
    move = phase7;
    flag = 0;
    flag_caido = 0;
    sleep(1);
  }
  

  ActuatorRequests request = RobotClient::buildRequestMessage(move);
  client.sendRequest(request);
}

void defense_position(RobotClient client){
  const char *move;
  const char *phase1 = "./defense_position/1.txt"; //
  const char *phase2 = "./defense_position/2.txt"; //


  SensorMeasurements sensors = client.receive();
  int time = sensors.time();

  if(stop_walk<5)
  {
    if(flag == 0) flag_time = time;
    flag++;

    if(time<(500+flag_time)) move = phase1;
    else
    {
      move = phase2;
      //flag = 0;
      //stop_walk++;
    }
  }

  ActuatorRequests request = RobotClient::buildRequestMessage(move);
  client.sendRequest(request);
}

int flagg = 0;

int valorint;

void read_accel(RobotClient client){
  string valor;

  SensorMeasurements sensors = client.receive();
  std::string printout;
  google::protobuf::TextFormat::PrintToString(sensors, &printout);

  if(flagg>10){
    if(flag_caido == 0) {
      string y = ("Y: ");
      size_t found = printout.find(y);
      valor = printout.substr(found+3,3);
      valorint = stoi(valor);
      flag_caido = 0;
    }
    if(valorint < 115) {
      flag_caido = 1;
      standup_back(client);
      
    }
    if(valorint > 140) {
      flag_caido = 1;
      standup_front(client);
    } 
    if (flag_caido != 1) walk(client);
    
    cout << printout << endl; 
  }

  flagg++;
  
}



int main(int argc, char *argv[]) {
    //  Socket to talk to clients
  void *context = zmq_ctx_new ();
  void *responder = zmq_socket (context, ZMQ_REP);
  int rc = zmq_bind (responder, "tcp://*:3737");
  assert (rc == 0);


  GOOGLE_PROTOBUF_VERIFY_VERSION;


  int port = -1;
  std::string host;

  int gyro_time_step = -1;
  int arg_idx = 1;
  int verbosity = 2;
  while (arg_idx < argc) {
    std::string current_arg(argv[arg_idx]);
    // Treating arguments
    if (current_arg[0] == '-') {
      if (current_arg == "-v") {
        if (arg_idx + 1 >= argc)
          usage("Missing value for verbosity");
        verbosity = std::stoi(argv[arg_idx + 1]);
        arg_idx++;
      } else  // current_arg == "-h" or "--help" or anything else
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

  int f = 0;

  const char *accel = "accel.txt"; 
  ActuatorRequests request = RobotClient::buildRequestMessage(accel);
  client.sendRequest(request);
  
  while (client.isOk()) {
    try {
      SensorMeasurements sensors = client.receive();

      char buffer [14];
      memset(buffer, 0, sizeof(buffer));
      zmq_send (responder, ".", 1, 0);      

      zmq_recv (responder, buffer, 14, 0);
      printf ("Game State: %s \n", buffer); 
      std::string buf = buffer;
      int zmq_msg_close (zmq_msg_t *buffer);
      

      if(buf == "initial") zmq_send(responder, "listening", 9, 0);

      if(buf == "ready" || buf == "kickoff opp" || buf == "kickoff Rob" || buf == "set") 
      {
        zmq_send(responder, "walk", 4, 0); 
        read_accel(client);             
      }


      if(buf == "penalty nosso") 
      {
        zmq_send (responder, "batendo penalty", 15, 0); 
        penalty(client);
                
      }
      if(buf == "penalty deles") 
      {
        zmq_send (responder, "defendendo penalty", 18, 0); 
        defense_position(client);
                
      }

      //walk(client);

      //standup_front(client); 
      //standup_back(client);    //nao funcionando 
      //kick_right_weak(client);
      //kick_left_weak(client);
      //turn_left(client);
      //turn_right(client);
      //penalty(client);
      //fall_left(client);
      //fall_right(client); 
      
      cout << flag_caido << endl;


    } catch (const std::runtime_error &exc) {
      std::cerr << "Runtime error: " << exc.what() << std::endl;
    }
  }

  return 0;
}

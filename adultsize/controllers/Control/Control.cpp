  /*--------------------------------------------------------------------

******************************************************************************
* @file control.cpp
* @author Isaac Jesus da Silva - ROBOFEI-HT - FEI 😛
* @version V2.2.0
* @created 20/01/2015
* @Modified 30/03/2021
* @e-mail isaac25silva@yahoo.com.br
* @brief control 😛
****************************************************************************
****************************************************************************
Arquivo fonte contendo o programa que controla os servos do corpo do robô
---------------------------------------------------------------------------*/
#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <unistd.h> //sleep, usleep
#include <libgen.h> //dirname
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <fstream>
#include <time.h>
#include <fcntl.h>
#include <termios.h>


//#include "dynamixel_sdk.h" //biblioteca SDK
#include <vector>

#include "minIni.h"
#include <string>

#include "MX28.h"
#include "MotionManager.h"
#include "LinuxMotionTimer.h"
#include <blackboard.h>
//#include <boost/program_options.hpp> //tratamento de argumentos linha de comando
#include "ActionMove.hpp"
#include "GaitMove.hpp"
#include <webots/Robot.hpp>

#ifdef MX28_1024
#define MOTION_FILE_PATH    "./Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "./Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "./Data/config.ini" 

#define DEBUG_PRINT true

#define LIMITE_TEMP uint8_t(80)    // Define a temperatura maxima dos motores

//------------- SDK ADD -----------------------------------
// Control table address -> MX28.h

// Protocol version
#define PROTOCOL_VERSION                2.0                // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        1000000
//#define DEVICENAME                      "/dev/robot/body"      // Check which port is being used on your controller
//------------- SDK ADD -----------------------------------

using namespace Robot;
using namespace std;

int kbhit(); //Function kbhit.cpp

//int check_servo(CM730 *cm730, int idServo, bool &stop_gait);
//int check_servo(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

//int Initialize_servo(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

void logInit();

//void PidMotion(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

//void PidStatic(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void sighandler(int sig)
{
    cout<< "\nProgram being closed!" << endl;
    exit(1);
}

void pythonRun(char **argv)
{
    wchar_t *program = Py_DecodeLocale(argv[0], NULL);
    if (program == NULL) {
        fprintf(stderr, "Fatal error: cannot decode argv[0]\n");
        exit(1);
    }
    Py_SetProgramName(program);  /* optional but recommended */
    Py_Initialize();
    PyRun_SimpleString("from time import time,ctime\n"
                       "print('Today is', ctime(time()))\n");
    if (Py_FinalizeEx() < 0) {
        exit(120);
    }
    PyMem_RawFree(program);
}

int main(int argc, char **argv)
{   

    pythonRun(argv);

    change_current_dir();
    
    minIni* ini;
    ini = new minIni((char *)INI_FILE_PATH);

    //Detecta o Ctrl+C-----------
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);
    //---------------------------

    //Acopla ou cria a memoria compartilhada
    int *mem = using_shared_memory(ini->getd("Communication","no_player_robofei",-1024) * 100); //0 for real robot 
    
    char string1[50]; //String
    bool stop_gait = true;
    //char *Servoport;
    //int idServo;
    bool flag_stop = 0;
    bool same_moviment = false;
    int buffer = 10000;
    unsigned int count_read=0;
    unsigned int step_time=8; // Determina a frequencia de leitura do blackboard
    //uint8_t dxl_error = 0;

    int flag = 0;

    //Configurando para prioridade maxima para executar este processo-------
    sprintf(string1,"echo password | sudo -S chrt -p -r 99 %d", getpid());
    system(string1); //prioridade
    
    printf( "\n===== ROBOFEI-HT Control Process Robot Teen =====\n\n");
    //-------------para entrada de argumentos-----------------------------------
//    namespace po=boost::program_options;

//    po::options_description desc("options");
//    desc.add_options()
//    ("help", "produce help message")
//    ("k", "Inicia com o controle do robô pelo teclado")
//    ("v", "Verifica a tensao nos servos do corpo")
//    ("t", "Verifica a temperatura dos servos do corpo")
//    ("g", "Inicia o controle para usar com a interface grafica")
//    ;

//    po::variables_map variables;
//    po::store(po::parse_command_line(argc, argv, desc), variables);
//    po::notify(variables);
    //--------------------------------------------------------------------------

    //////////////////// Framework Initialize ////////////////////////////
    // ---- Open USBDynamixel -----------------------------------------------{
    //DECLARAÇÃO SDK
    //dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    //dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

//    if(Initialize_servo(packetHandler, portHandler) == 1) // chama a função que encontra o endereço de comunicação com o servo
//        return 0;

    webots::Robot *robot = new webots::Robot();

    //LinuxCM730 linux_cm730(string1);
    //CM730 cm730(&linux_cm730);
    //if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    if(MotionManager::GetInstance()->Initialize(robot) == false)
    {
        printf("Fail to initialize Motion Manager!\n");
        return 0;
    }
    MotionManager::GetInstance()->memBB = mem;
    
    printf( "\n===== Inicializou o Motion Manager =====\n\n");

    //======================== check voltage ===========================================

//    if (variables.count("v")) //verifica se foi chamado o argumento de controle pelo teclado
//    {
//        uint16_t value = 0;
//        //if(cm730.ReadByte(12, MX28::P_PRESENT_VOLTAGE, &value, 0) != CM730::SUCCESS)
//        if(packetHandler->read2ByteTxRx(portHandler, 7, 144, &value, &dxl_error) !=  COMM_SUCCESS)
//        {
//          std::cout<<"Erro na leitura da tensao!"<<std::endl;
//        }
//        else
//        {
//          std::cout<<"Tensao = "<<float(value)/10<<"V"<<std::endl;
//        }
//        return 0;
//    }
    //==================================================================================
//if(packetHandler->read2ByteTxRx(portHandler, 7, 144, &value, &dxl_error) !=  COMM_SUCCESS)
    //======================== check temperature =======================================
//    if (variables.count("t")) //verifica se foi chamado o argumento de controle pelo teclado
//    {
//        uint8_t value = 0;
//        for(int id = 1; id < JointData::NUMBER_OF_JOINTS-2; id++)
//        {
//          //if(cm730.ReadByte(id, MX28::P_PRESENT_TEMPERATURE, &value, 0) != CM730::SUCCESS)
//          if(packetHandler->read1ByteTxRx(portHandler, id, 146, &value, &dxl_error) !=  COMM_SUCCESS)
//          {
//            std::cout<<"Erro na leitura da temperatura no motor "<<id<<std::endl;
//          }
//          else
//          {
//            std::cout<<"Temperatura motor "<< id<<" = "<<int(value)<<"°"<<std::endl;
//          }
//        }
//        return 0;
//    }
    //==================================================================================

    MotionManager::GetInstance()->LoadINISettings(ini);

    //Criando objeto da classe dos movimento de acoes----------------------------
    ActionMove actionMove(mem, (char *)MOTION_FILE_PATH);
    printf( "\n===== Carregou os parâmetros do config.ini =====\n\n");
    //**************************************************************************

    //Criando objeto da classe dos movimentos de caminhada----------------------
    GaitMove gaitMove(mem, ini);
    printf( "\n===== Criando objeto da classe dos movimentos de caminhada =====\n\n");
    //**************************************************************************
    

    MotionManager::GetInstance()->LoadINISettings(ini);
    Walking::GetInstance()->LoadINISettings(ini);
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    LinuxMotionTimer linuxMotionTimer;
    linuxMotionTimer.Initialize(MotionManager::GetInstance());
    linuxMotionTimer.Start();
    printf( "\n===== Iniciou a Thread =====\n\n");
    
    /////////////////////////////////////////////////////////////////////
    actionMove.poseStandup(stop_gait); /* Init(stand up) pose */
    
    //====== Reset the IMU ==========
//    sleep(2);
//    write_int(mem, IMU_RESET, 1);
//    sleep(1);
    //int k=0;

//    if (!variables.count("k")) //verifica se foi chamado o argumento de controle pelo teclado
//    {
//        //====== Reset the IMU ==========
//        while(read_float(mem, IMU_EULER_Z) > 0.0005 || read_float(mem, IMU_EULER_Z) < -0.0005)
//        {
//            sleep(2);
//            write_int(mem, IMU_RESET, 1);
//            cout<<"Reseting IMU"<<endl;
//            sleep(1); 
//            if (k>4)
//            {
//                cout<<"Error: reset IMU failed"<<endl;
//                break;
//            }
//            k++;
//        }
//    } 
    //===============================

    /*//
    MotionStatus::m_CurrentJoints.SetValue(1,1);
    MotionStatus::m_CurrentJoints.SetValue(2,1);
    MotionStatus::m_CurrentJoints.SetValue(3,1);
    MotionStatus::m_CurrentJoints.SetValue(4,1);
    MotionStatus::m_CurrentJoints.SetValue(5,1);
    MotionStatus::m_CurrentJoints.SetValue(6,1);
    MotionStatus::m_CurrentJoints.SetValue(7,1);
    MotionStatus::m_CurrentJoints.SetValue(8,1);
    MotionStatus::m_CurrentJoints.SetValue(9,1);
    /*/



    int key = 102; // greeting 104   // walk_foward_slow 107 
    //***********************************************************************************************
    if (true) //verifica se foi chamado o argumento de controle pelo teclado
    {
    //-------------iniciando o modulo de andar pelo teclado------------------------------------------
        buffer=0;
        while(1)
        {
            //int key = kbhit();
            
            usleep(20*1000);
            

            //mantem o key com valor da tecla f ou k para realizar o soft_starter-----
            if(key!=0)
            {
                if(key == 102 || key == 107)
                    buffer=key;
                else
                    buffer=0;
                   same_moviment=false;
                //Altera o torque dos motores pelos valores do PID
//                if(key==116)
//                    PidStatic(packetHandler, portHandler);
//                else
//                    PidMotion(packetHandler, portHandler);
            }
            else
            {
                key=buffer;
                same_moviment=true;
            }
            //-------------------------------------------------------------------------

            switch(key)
            {
                case 97: //a
//                    PidMotion(packetHandler, portHandler);
                    actionMove.standupFront(stop_gait);
//                    PidStatic(packetHandler, portHandler);
                break;

                case 98: //b
                    actionMove.standupBack(stop_gait);
                break;

                case 112: //p 
                    actionMove.kick_right_strong(robot, stop_gait);
                break;

                case 108: //l
                    actionMove.kick_left_strong(robot, stop_gait);
                break;

                case 99: //c
                    actionMove.kick_right_weak(stop_gait);
                break;

                case 103: //g
                    actionMove.kick_left_weak(stop_gait);
                break;

                case 102: //f
                    gaitMove.walk_foward_fast(stop_gait, same_moviment);
                break;

                case 100: //d
                    gaitMove.turn_right(stop_gait, true, same_moviment);
                break;

                case 105: //i
                    actionMove.pass_left(robot, stop_gait);
                break;

                case 101: //e
                    gaitMove.turn_left(stop_gait, true, same_moviment);
                break;

                case 106: //j
                    actionMove.pass_right(robot, stop_gait);
                break;

                case 109: //m
                    gaitMove.sidle_left(stop_gait, same_moviment);
                break;

                case 110: //n
                    gaitMove.sidle_right(stop_gait, same_moviment);
                break;

                case 111: //o
                    gaitMove.turn_around_ball_left(stop_gait, same_moviment);
                break;

                case 113: //q
                    gaitMove.turn_around_ball_right(stop_gait, same_moviment);
                break;

                case 107: //k
                    /*if(flag<=3050) 
                    {
                        gaitMove.walk_foward_slow(stop_gait, true, same_moviment);
                        cout << flag << endl;
                        flag++;
                    }
                    else if (flag<=3350)
                    {
                        gaitMove.turn_left(stop_gait, true, same_moviment);
                        cout << flag << endl;
                        flag++;
                    }
                    else 
                    {
                        
                        gaitMove.robot_stop(stop_gait);
                        
                    }
                    */
                    gaitMove.walk_foward_slow(stop_gait, true, same_moviment);
                    

                    
                    
                break;

                case 114: //r
                    gaitMove.walk_backward_slow(stop_gait, true, same_moviment);
                break;

                case 118: //v
                    gaitMove.walk_backward_fast(stop_gait, same_moviment);
                break;

                case 115: //s
                    gaitMove.Gait_in_place(stop_gait, same_moviment);
                break;

                case 116: //t
                    gaitMove.robot_stop(stop_gait);
                break;

                case 117: //u
                    actionMove.goalkeeper(stop_gait);
                break;

                case 104: //h
//                    PidMotion(packetHandler, portHandler);
                    actionMove.greetings(stop_gait);
//                    PidStatic(packetHandler, portHandler);
                break;

                case 119: //w
//                    PidMotion(packetHandler, portHandler);
                 break;

                case 122: //z
                    actionMove.goodBye(stop_gait);
                 break;

                case 120: //x
                    gaitMove.walk_foward_fast_direct(stop_gait, same_moviment);
                break;

                case 27: //ESC (stop)
                    cout << " | Exit process" << endl;
                    return 0;
                break;

                default:
                    if(key!=0)
                        cout<< " | \e[1;31mTecla incorreta - verifique quais teclas controlam o robo\e[0m"<<endl;
                break;

            }
/*
          //if (Action::GetInstance()->IsRunning()==0 && Walking::GetInstance()->IsRunning()==0 && check_servo(&cm730, idServo, stop_gait)!=0)
          if ((Action::GetInstance()->IsRunning()==0 && Walking::GetInstance()->IsRunning()==0 && check_servo(packetHandler, portHandler))!=0)
                return 0;
*/
        }
    }
    //==========================================================================================


    //***************************************************************************************
    //-------------------------Controle pela decisão-----------------------------------------
    //logInit(); // save the time when start the control process
    while(1)
    {       
            //Confere se o movimento atual e o mesmo do anterior----------
            if(buffer==read_int(mem, DECISION_ACTION_A))
                same_moviment = true;
            else
            {
                same_moviment = false;
                std::cout<< "\nAction " << read_int(mem, DECISION_ACTION_A); // Mostra o valor da ação
                count_read=0;
                //Altera o torque dos motores pelos valores do PID
//                if(read_int(mem, DECISION_ACTION_A) == 0)
//                    PidStatic(packetHandler, portHandler);
//                else
//                    PidMotion(packetHandler, portHandler);
            }
            buffer = read_int(mem, DECISION_ACTION_A);
            //------------------------------------------------------------

            if (read_int(mem, IMU_STATE)){ // Ve se esta caido
                if(read_float(mem, IMU_ACCEL_X) > 0){  //Levanta se caido de frente
                    actionMove.standupFront(stop_gait);
                }
                else{  //Levanta se caido de costa
                    actionMove.standupBack(stop_gait);
                }
                stop_gait = 1;
                sleep(1);
                flag_stop = false;
            }


            if(read_int(mem, DECISION_ACTION_A) == 0)
            {
                if(flag_stop==false)
                    gaitMove.robot_stop(stop_gait);
                flag_stop = true; //variavel que indica que o robo ja estava parado, isso evita de repetir o movimento
            }
            else
                flag_stop = false;

            if(read_int(mem, DECISION_ACTION_A) == 1)
                gaitMove.walk_foward_fast(stop_gait, same_moviment);

            if(read_int(mem, DECISION_ACTION_A) == 2)
                gaitMove.turn_left(stop_gait, true, same_moviment);

            if(read_int(mem, DECISION_ACTION_A) == 3)
                gaitMove.turn_right(stop_gait, true, same_moviment);

            if(read_int(mem, DECISION_ACTION_A) == 4)
                actionMove.kick_right_strong(robot, stop_gait);

            if(read_int(mem, DECISION_ACTION_A) == 5)
                actionMove.kick_left_strong(robot, stop_gait);

            if(read_int(mem, DECISION_ACTION_A) == 6)
                gaitMove.sidle_left(stop_gait, same_moviment);

            if(read_int(mem, DECISION_ACTION_A) == 7)
                gaitMove.sidle_right(stop_gait, same_moviment);

            if(read_int(mem, DECISION_ACTION_A) == 8)
                gaitMove.walk_foward_slow(stop_gait, true, same_moviment);

            if(read_int(mem, DECISION_ACTION_A) == 9)
                gaitMove.turn_around_ball_left(stop_gait, same_moviment);

            if(read_int(mem, DECISION_ACTION_A) == 10)
                actionMove.goalkeeper(stop_gait);

            if(read_int(mem, DECISION_ACTION_A) == 11)
                gaitMove.Gait_in_place(stop_gait, same_moviment);

            if(read_int(mem, DECISION_ACTION_A) == 12)
                //actionMove.pass_left(&cm730, stop_gait);

            if(read_int(mem, DECISION_ACTION_A) == 13)
                //actionMove.pass_right(&cm730, stop_gait);

            if(read_int(mem, DECISION_ACTION_A) == 14)
                gaitMove.turn_around_ball_right(stop_gait, same_moviment);

            if(read_int(mem, DECISION_ACTION_A) == 15)
            {
                if (read_int(mem, IMU_STATE))// check if robot is fall
                    actionMove.standupFront(stop_gait);
                else
                    std::cout<<" | \e[1;31mRobô não está caido ou IMU está desligada\e[0m"<<std::endl;
            }
            if(read_int(mem, DECISION_ACTION_A) == 16)
            {
                if (read_int(mem, IMU_STATE))// check if robot is fall
                    actionMove.standupBack(stop_gait);
                else
                    std::cout<<" | \e[1;31mRobô não está caido ou IMU está desligada\e[0m"<<std::endl;
            }
            if(read_int(mem, DECISION_ACTION_A) == 17)
            {
                gaitMove.walk_backward_fast(stop_gait, same_moviment);
            }
            if(read_int(mem, DECISION_ACTION_A) == 18)
            {
                gaitMove.walk_backward_slow(stop_gait, true, same_moviment);
            }

            if(read_int(mem, DECISION_ACTION_A) == 19)
                actionMove.greetings(stop_gait);

            if(read_int(mem, DECISION_ACTION_A) == 20)
                actionMove.goodBye(stop_gait);

            if(read_int(mem, DECISION_ACTION_A) == 21)
                actionMove.kick_right_weak(stop_gait); //Chute fraco com pe direito

            if(read_int(mem, DECISION_ACTION_A) == 22)
                actionMove.kick_left_weak(stop_gait); //Chute fraco com pe esquerdo

            // Escreve na variável de telemetria.
            write_int(mem, CONTROL_WORKING, 1);

            //Imprime na tela o tempo que esta ocioso por nao receber uma nova instrucao da decisao-------
            count_read++;
            std::cout << "\rReading BlackBoard" <<"[\e[38;5;82m"<< count_read<<"\e[0m] | Tempo ocioso"<<"[\e[38;5;82m"<< count_read*step_time/1000<<"s\e[0m]";
            fflush (stdout);
            usleep(step_time*1000); //Operando na frequencia de 1/step_time Hertz
            //--------------------------------------------------------------------------------------------
    }
    //--------------------------------------------------------------------------------------------------
    //==================================================================

    std::cout<<"Press some key to end!\n"<<std::endl;
    getchar();

    //LinuxActionScript::ScriptStart("script.asc");
    //while(LinuxActionScript::m_is_running == 1) sleep(10);

    return 0;
}
//////////////////// Framework Initialize ////////////////////////////
// ---- Open USBDynamixel -----------------------------------------------{
//int Initialize_servo(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler)

//{
//  bool servoConectado = false;
//  int idServo;
//  char string1[50];
//  sprintf(string1,"/dev/robot/body");
///*
//  LinuxCM730* linux_cm730;
//  linux_cm730 = new LinuxCM730(string1);
//  CM730* cm730;
//  cm730 = new CM730(linux_cm730);
//*/

//  std::vector<uint8_t> vec;
//  int dxl_comm_result = COMM_TX_FAIL;

//  //OPEN PORT
//  if (portHandler->openPort())
//  {
//    printf("Succeeded to open the port!\n");
//  }
//  else
//  {
//    printf("Failed to open the port!\n");
//    return 1;
//  }

//  // Set port baudrate
//  if (portHandler->setBaudRate(BAUDRATE))
//  {
//    printf("Succeeded to change the baudrate!\n");
//  }
//  else
//  {
//    printf("Failed to change the baudrate!\n");
//    return 1;
//  }

//  if( MotionManager::GetInstance()->Initialize(packetHandler, portHandler) == 0)
//  { // not connect with board rs485
//    std::cout<<"\e[1;31mNão há nenhuma placa USB/RS-485 conectada no computador.\n\n\e[0m"<<std::endl;
//		return -1;
//  }
//  else
//  {
//    dxl_comm_result = packetHandler->broadcastPing(portHandler, vec);
//    if (dxl_comm_result != COMM_SUCCESS)
//    {
////     for (int a = 0; a <101; a++)
////       {if (dxl_comm_result != COMM_SUCCESS)
////        {
////          cout<<"tentativa "<<a<<" de detecção"<<endl;
////          usleep(500000);
//////          return 1;
////        }
////       }  
//      printf("\e[0;31mConectou-se a placa USB/RS-485 mas não conseguiu se comunicar com nenhum servo.\e[0m\n");
//      std::cout<<"Endereço: "<<"/dev/robot/body"<<std::endl;
//      std::cout<<"\e[0;36mVerifique se a chave que liga os servos motores está na posição ligada.\n\n\e[0m"<<std::endl;
//      return 1;
//    }
//    else
//    {
//      printf("Connected and communicating with the body of the robot: \n");
//      for (int i = 0; i < (int)vec.size(); i++)
//      {
//        printf("[ID:%03d]\n", vec.at(i));
//      }
//    }
//  }
//}
/*
  		for(int id=1; id<19; id++) //check communicating
  		{
  		    cm730->ReadByte(id, MX28::P_ID, &idServo, 0); // Read the servo id of servo 1
  		    servoConectado = idServo == id;
  		    if(servoConectado)
  		    {
  		        std:: cout<<"Connected and communicating with the body of the robot!\n";
  		        return 0;
  		    }
  			usleep(1000);
  		}

    }
    printf("\e[0;31mConectou-se a placa USB/RS-485 mas não conseguiu se comunicar com nenhum servo.\e[0m\n");
    std::cout<<"Endereço: "<<"/dev/robot/body"<<std::endl;
    std::cout<<"\e[0;36mVerifique se a chave que liga os servos motores está na posição ligada.\n\n\e[0m"<<std::endl;
    return 1;
}
*/

////int check_servo(dynamixel::PacketHandler *cm730, int idServo, bool &stop_gait)
//int check_servo(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler)
//{
//    static uint8_t i=0;
//    uint8_t save=-1;
//    uint8_t dxl_error = 0;
//    int dxl_comm_result = COMM_TX_FAIL;
//    std::vector<uint8_t> vec;
//    dxl_comm_result = packetHandler->broadcastPing(portHandler, vec);


//    //for(int erro,j=1 ; i==0 && j<=18; j++)
//    for(int j=1 ; i==0 && j<=18; j++)
//    {
//        //cm730->WriteByte(j, MX28::P_HIGH_LIMIT_TEMPERATURE, LIMITE_TEMP, &erro);
//        packetHandler->write1ByteTxRx(portHandler, j, MX28::P_HIGH_LIMIT_TEMPERATURE, LIMITE_TEMP, &dxl_error);
//    }

//    i++;
//    if (i==JointData::NUMBER_OF_JOINTS-2) //Ultimo motor: 18
//        i=1;

//    if (i<=6)// Membro superiores ate 6
//    {
//        //if(cm730->ReadWord(i, MX28::P_TORQUE_LIMIT, &save, 0)!=0)
//        //if(packetHandler->read2ByteTxRx(portHandler, i, MX28::P_GOAL_CURRENT, &save, &dxl_error) != 0)

//        if (dxl_comm_result != COMM_SUCCESS)
//        {
//            cout<<"Perda na comunicação com o motor "<<i<<" - Membro superior"<<endl;
//            usleep(500000);
//            return 0;
//        }

//        if (save<=0)// Testa o torque
//        {
//            //if(cm730->ReadWord(i, MX28::P_PRESENT_TEMPERATURE, &save, 0)!=0)
//            if(packetHandler->read1ByteTxRx(portHandler, i, MX28::P_PRESENT_TEMPERATURE, &save, &dxl_error) != 0)
//            {
//                cout<<"Perda na comunicação com o motor "<<i<<" - Membro superior"<<endl;
//                usleep(500000);
//                return 0;
//            }

//            if(save>=LIMITE_TEMP)
//            {
//                cout<<"Motor "<<i<<" aqueceu a " << save << ", motor desligado - Membro superior"<<endl;
//                usleep(500000);
//                return 0;
//            }
//            else
//            {
//                cout<<"Motor "<<i<<" excedeu torque, motor desligado - Membro superior"<<endl;
//                usleep(500000);
//                return 0;
//            }
//        }
//    }
//    else{ // Membro inferiores, do 7 em diante
//    uint16_t save;
//        //if(cm730->ReadWord(i, MX28::P_TORQUE_LIMIT_L, &save, 0)!=0)
//        if(packetHandler->read2ByteTxRx(portHandler, i, MX28::P_GOAL_CURRENT, &save, &dxl_error) != 0)
//        {

//            cout<<"Perda na comunicação com o motor "<<i<<" - Membro inferior"<<endl;
//            usleep(500000);
//            return 0;
//        }

//        if (save<=0)// Testa o torque
//        {
//            uint8_t save;
//            //if(cm730->ReadWord(i, MX28::P_PRESENT_TEMPERATURE, &save, 0)!=0)

//            if(packetHandler->read1ByteTxRx(portHandler, i, MX28::P_PRESENT_TEMPERATURE, &save, &dxl_error) != 0)
//            {
//                cout<<"Perda na comunicação com o motor "<<i<<" - Membro inferior"<<endl;
//                usleep(500000);
//                return 0;
//            }

//            if(save>=LIMITE_TEMP)
//            {
//                cout<<"Motor "<<i<<" aqueceu a " << save << ", motor desligado - Membro inferior"<<endl;
//                usleep(500000);
//                return 0;
//            }
//            else
//            {
//                cout<<"Motor "<<i<<" excedeu torque, motor desligado - Membro inferior"<<endl;
//                usleep(500000);
//                return 0;
//            }
//        }
//    }

//    return 0;
//}

void logInit()
{
        std::fstream File;
        time_t _tm =time(NULL);
        struct tm * curtime = localtime ( &_tm );
        File.open("Control.log", std::ios::app | std::ios::out);
        if (File.good() && File.is_open())
        {
            File << "Inicializando o processo do controle "<<" --- ";
            File << asctime(curtime);
            File.flush();
            File.close();
        }
        else
	    printf("Erro ao Salvar o arquivo\n");
}

//void PidMotion(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler)
//{
//  for(int l = 1; l <=18 ;l++)
//  {
////    cout<<"Torque alto"<<endl;
//    uint8_t dxl_error = 0;
//    packetHandler->write2ByteTxRx(portHandler, l, MX28::P_POSITION_D_GAIN, 3000, &dxl_error);
//    packetHandler->write2ByteTxRx(portHandler, l, MX28::P_POSITION_I_GAIN, 0, &dxl_error);
//    packetHandler->write2ByteTxRx(portHandler, l, MX28::P_POSITION_P_GAIN, 2000, &dxl_error);
//    packetHandler->write2ByteTxRx(portHandler, l, MX28::P_GOAL_PWM, 885, &dxl_error);
//  }
//}

//void PidStatic(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler)
//{
////    cout<<"Torque baixo"<<endl;
//    uint8_t dxl_error = 0;
//  for(int l = 1; l <=18 ;l++)
//  {
//    packetHandler->write2ByteTxRx(portHandler, l, MX28::P_POSITION_D_GAIN, 1000, &dxl_error);
//    packetHandler->write2ByteTxRx(portHandler, l, MX28::P_POSITION_I_GAIN, 0, &dxl_error);
//    packetHandler->write2ByteTxRx(portHandler, l, MX28::P_POSITION_P_GAIN, 1000, &dxl_error);
//    //packetHandler->write2ByteTxRx(portHandler, l, MX28::P_GOAL_PWM, 850, &dxl_error);
//  }
//}

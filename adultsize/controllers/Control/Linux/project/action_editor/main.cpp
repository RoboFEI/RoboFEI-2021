/*--------------------------------------------------------------------------

****************************************************************************
* @file main.cpp
* @author Isaac Jesus da Silva - ROBOFEI-HT - FEI
* @version V0.0.5
* @created 20/01/2015
* @Modified 30/09/2016
* @e-mail isaac25silva@yahoo.com.br
* @brief Action Editor
****************************************************************************

Arquivo fonte contendo o programa que grava pontos de ações do robô

/-------------------------------------------------------------------------*/


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <term.h>
#include <ncurses.h>
#include <libgen.h>
#include <signal.h>
#include "cmd_process.h"
#include "blackboard.h"
#include <stdlib.h>     /* system, NULL, EXIT_FAILURE */



#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"


#define INI_FILE_PATH       "../../../Data/config.ini"

//------------- SDK ADD -----------------------------------
// Control table address -> MX28.h

// Protocol version
#define PROTOCOL_VERSION                2.0                // See which protocol version is used in the Dynamixel

// Default setting
#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/robot/body"      // Check which port is being used on your controller
//------------- SDK ADD -----------------------------------

using namespace Robot;

LinuxMotionTimer linuxMotionTimer;


void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void sighandler(int sig)
{
    struct termios term;
    tcgetattr( STDIN_FILENO, &term );
    term.c_lflag |= ICANON | ECHO;
    tcsetattr( STDIN_FILENO, TCSANOW, &term );

    exit(0);
}

void PidMotion(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

void PidStatic(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);


//////////////////// Framework Initialize ////////////////////////////
// ---- Open USBDynamixel -----------------------------------------------{
// int Initialize_servo(char *string1)
int Initialize_servo(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler)
{
  bool servoConectado = false;
  int idServo;
  char string1[50];
  sprintf(string1,"/dev/robot/body");
/*
  LinuxCM730* linux_cm730;
  linux_cm730 = new LinuxCM730(string1);
  CM730* cm730;
  cm730 = new CM730(linux_cm730);
*/

  std::vector<uint8_t> vec;
  int dxl_comm_result = COMM_TX_FAIL;

  //OPEN PORT
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    return 1;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    return 1;
  }

  if( MotionManager::GetInstance()->Initialize(packetHandler, portHandler) == 0)
  { // not connect with board rs485
    std::cout<<"\e[1;31mNão há nenhuma placa USB/RS-485 conectada no computador.\n\n\e[0m"<<std::endl;
		return -1;
  }
  else
  {
    dxl_comm_result = packetHandler->broadcastPing(portHandler, vec);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      printf("\e[0;31mConectou-se a placa USB/RS-485 mas não conseguiu se comunicar com nenhum servo.\e[0m\n");
      std::cout<<"Endereço: "<<"/dev/robot/body"<<std::endl;
      std::cout<<"\e[0;36mVerifique se a chave que liga os servos motores está na posição ligada.\n\n\e[0m"<<std::endl;
      return 1;
    }
    else
    {
      printf("Connected and communicating with the body of the robot: \n");
      for (int i = 0; i < (int)vec.size(); i++)
      {
        printf("[ID:%03d]\n", vec.at(i));
      }
    }
  }
}


int main(int argc, char *argv[])
{
    int ch;
    char filename[128];
	  char string1[50]; //String

    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini;
    ini = new minIni((char *)INI_FILE_PATH);

   //Acopla ou cria a memoria compartilhada
    int *mem = using_shared_memory(ini->getd("Communication","no_player_robofei",-1024) * 100); //0 for real robot

    if(argc < 2)
        strcpy(filename, MOTION_FILE_PATH); // Set default motion file path
    else
        strcpy(filename, argv[1]);

    //Configurando para prioridade maxima para executar este processo-------
    sprintf(string1,"echo fei 123456| sudo -S renice -20 -p %d", getpid());
    system(string1);//prioridade

    /////////////// Load/Create Action File //////////////////
    if(Action::GetInstance()->LoadFile(filename) == false)
    {
        printf("Can not open \e[0;31m%s\e[0m\n", filename);
        printf("Do you want to make a new action file? (y/n) ");
        ch = _getch();
        if(ch != 'y')
        {
            printf("\n");
            return 0;
        }

        if(Action::GetInstance()->CreateFile(filename) == false)
        {
            printf("\e[1;31mFail to create %s\e[0m\n", filename);
            return 0;
        }
    }
    ////////////////////////////////////////////////////////////

    //////////////////// Framework Initialize ////////////////////////////
    // ---- Open USBDynamixel -----------------------------------------------{
    //DECLARAÇÃO SDK
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    Initialize_servo(packetHandler, portHandler); // chama a função que encontra o endereço de comunicação com o servo

    // Initialize_servo(string1); // chama a função que encontra o endereço de comunicação com o servo
    // LinuxCM730 linux_cm730(string1);
    // CM730 cm730(&linux_cm730);
    if(MotionManager::GetInstance()->Initialize(packetHandler, portHandler) == false)
    {
        printf("Fail to initialize Motion Manager!\n");
    }
    sleep(1);
    MotionManager::GetInstance()->memBB = mem;
   //==================================================================================


    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    linuxMotionTimer.Initialize(MotionManager::GetInstance());
	  linuxMotionTimer.Stop();
	//MotionManager::GetInstance()->StopThread();
    /////////////////////////////////////////////////////////////////////
    
    // DrawIntro(&cm730);
    DrawIntro(packetHandler, portHandler);
    // printf("cade3");

    while(1)
    {
        ch = _getch();

        if(ch == 0x1b)
        {
            ch = _getch();
            if(ch == 0x5b)
            {
                ch = _getch();
                if(ch == 0x41)      // Up arrow key
                    MoveUpCursor();
                else if(ch == 0x42) // Down arrow key
                    MoveDownCursor();
                else if(ch == 0x44) // Left arrow key
                    MoveLeftCursor();
                else if(ch == 0x43) // Right arrow key
                    MoveRightCursor();
            }
        }
        else if( ch == '[' )
            UpDownValue(packetHandler, portHandler, 1);
        else if( ch == '{' )
            UpDownValue(packetHandler, portHandler, -10);
        else if( ch == '}' )
            UpDownValue(packetHandler, portHandler, 10);
        else if( ch == ' ' )
            ToggleTorque(packetHandler, portHandler);
        else if( ch >= 'A' && ch <= 'z' )
        {
            char input[128] = {0,};
            char *token;
            int input_len;
            char cmd[80];
            int num_param;
            int iparam[30];

            int idx = 0;

            BeginCommandMode();

            printf("%c", ch);
            input[idx++] = (char)ch;

            while(1)
            {
                ch = _getch();
                if( ch == 0x0A )
                    break;
                else if( ch == 0x7F )
                {
                    if(idx > 0)
                    {
                        ch = 0x08;
                        printf("%c", ch);
                        ch = ' ';
                        printf("%c", ch);
                        ch = 0x08;
                        printf("%c", ch);
                        input[--idx] = 0;
                    }
                }
                else if( ( ch >= 'A' && ch <= 'z' ) || ch == ' ' || ch == '-' || ( ch >= '0' && ch <= '9'))
                {
                    if(idx < 127)
                    {
                        printf("%c", ch);
                        input[idx++] = (char)ch;
                    }
                }
            }

            fflush(stdin);
            input_len = strlen(input);
            if(input_len > 0)
            {
                token = strtok( input, " " );
                if(token != 0)
                {
                    strcpy( cmd, token );
                    token = strtok( 0, " " );
                    num_param = 0;
                    while(token != 0)
                    {
                        iparam[num_param++] = atoi(token);
                        token = strtok( 0, " " );
                    }

                    if(strcmp(cmd, "exit") == 0)
                    {
                        if(AskSave() == false)
                            break;
                    }
                    else if(strcmp(cmd, "re") == 0)
                        DrawPage();
                    else if(strcmp(cmd, "help") == 0)

                        HelpCmd();
                    else if(strcmp(cmd, "n") == 0)
                        NextCmd();
                    else if(strcmp(cmd, "b") == 0)
                        PrevCmd();
                    else if(strcmp(cmd, "time") == 0)
                        TimeCmd();
                    else if(strcmp(cmd, "speed") == 0)
                        SpeedCmd();
                    else if(strcmp(cmd, "page") == 0)
                    {
                        if(num_param > 0)
                            PageCmd(iparam[0]);
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "play") == 0)
                    {
                        PidMotion(packetHandler, portHandler);
                        PlayCmd(packetHandler, portHandler);
                        PidStatic(packetHandler, portHandler);
                    }
                    else if(strcmp(cmd, "set") == 0)
                    {
                        if(num_param > -900)
                            SetValue(packetHandler, portHandler, iparam[0]);
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "list") == 0)
                        ListCmd();
                    else if(strcmp(cmd, "on") == 0)
                        OnOffCmd(packetHandler, portHandler, true, num_param, iparam);
                    else if(strcmp(cmd, "off") == 0)
                        OnOffCmd(packetHandler, portHandler, false, num_param, iparam);
                    else if(strcmp(cmd, "w") == 0)
                    {
                        if(num_param > 0)
                            WriteStepCmd(iparam[0]);
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "d") == 0)
                    {
                        if(num_param > 0)
                            DeleteStepCmd(iparam[0]);
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "i") == 0)
                    {
                        if(num_param == 0)
                            InsertStepCmd(0);
                        else
                            InsertStepCmd(iparam[0]);
                    }
                    else if(strcmp(cmd, "m") == 0)
                    {
                        if(num_param > 1)
                            MoveStepCmd(iparam[0], iparam[1]);
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "copy") == 0)
                    {
                        if(num_param > 0)
                            CopyCmd(iparam[0]);
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "new") == 0)
                        NewCmd();
                    else if(strcmp(cmd, "g") == 0)
                    {
                        if(num_param > 0)
                        {
                            PidMotion(packetHandler, portHandler);
                            GoCmd(packetHandler, portHandler, iparam[0]);
                            PidStatic(packetHandler, portHandler);
                        }
                        else
                            PrintCmd("Need parameter");
                    }
                    else if(strcmp(cmd, "save") == 0)
                        SaveCmd();
                    else if(strcmp(cmd, "name") == 0)
                        NameCmd();
                    else if(strcmp(cmd, "t") == 0)
					{
					    PidMotion(packetHandler, portHandler);
						goInitPage();
						PlayCmd(packetHandler, portHandler);
						backToPage();
						PidStatic(packetHandler, portHandler);
					}
                    else if(strcmp(cmd, "read") == 0)
						readServo(packetHandler, portHandler);
					else if(strcmp(cmd, "z") == 0)
						PidMotion(packetHandler, portHandler);
					else if(strcmp(cmd, "x") == 0)
						PidStatic(packetHandler, portHandler);
					else
                        PrintCmd("Bad command! please input 'help'");
                }
            }

            EndCommandMode();
        }
    }

    DrawEnding();

    return 0;
}


void PidMotion(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler)
{
  for(int l = 1; l <=18 ;l++)
  {
//    cout<<"Torque alto"<<endl;
    uint8_t dxl_error = 0;
    packetHandler->write2ByteTxRx(portHandler, l, MX28::P_POSITION_D_GAIN, 3000, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, l, MX28::P_POSITION_I_GAIN, 0, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, l, MX28::P_POSITION_P_GAIN, 2000, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, l, MX28::P_GOAL_PWM, 885, &dxl_error);
  }
}

void PidStatic(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler)
{
//    cout<<"Torque baixo"<<endl;
    uint8_t dxl_error = 0;
  for(int l = 1; l <=18 ;l++)
  {
    packetHandler->write2ByteTxRx(portHandler, l, MX28::P_POSITION_D_GAIN, 1000, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, l, MX28::P_POSITION_I_GAIN, 0, &dxl_error);
    packetHandler->write2ByteTxRx(portHandler, l, MX28::P_POSITION_P_GAIN, 1000, &dxl_error);
    //packetHandler->write2ByteTxRx(portHandler, l, MX28::P_GOAL_PWM, 850, &dxl_error);
  }
}




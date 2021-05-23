/*--------------------------------------------------------------------

******************************************************************************
* @file ActionMove.cpp
* @author Isaac Jesus da Silva - ROBOFEI-HT - FEI 😛
* @version V1.0.5
* @created 14/07/2016
* @Modified 15/07/2016
* @e-mail isaac25silva@yahoo.com.br
* @revisor Isaac Jesus da Silva
* @e-mail isaac25silva@yahoo.com.br
* @brief action move 😛
****************************************************************************
****************************************************************************
Arquivo fonte contendo metodos de movimentos de acoes
/--------------------------------------------------------------------------*/

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include "MotionManager.h"
#include "ActionMove.hpp"
#include <webots/Robot.hpp>

ActionMove::ActionMove(int *mem_t, char* filename)
{
    mem = mem_t;
    dxl_error = 0;
    //this->portHandler = portHandler;
    Action::GetInstance()->LoadFile(filename);
    Action::GetInstance()->Initialize();
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
}

//========================================================================
//Execute the move action-------------------------------------------------
void ActionMove::move_action(int move_number, bool interrupt, bool &stop_gait)
{
    //write_int(mem, CONTROL_MOVING, 1);
    
    while(Walking::GetInstance()->GetCurrentPhase()!=0 && Walking::GetInstance()->IsRunning()!=0)  usleep(8*1000);
    Walking::GetInstance()->Stop();
    Walking::GetInstance()->m_Joint.SetEnableBody(false);
    Action::GetInstance()->m_Joint.SetEnableBody(true);
    MotionManager::GetInstance()->SetEnable(true);
    Action::GetInstance()->Start(move_number); // Realiza a ação do numero contido no move_number
    while(Action::GetInstance()->IsRunning() && !interrupt) usleep(8*1000); // Aguarda finalizar a ação ou para por interrupção
    stop_gait = 1;
    //write_int(mem, CONTROL_MOVING, 0);
}


//==============================================================================
void ActionMove::pass_left(webots::Robot *robot, bool &stop_gait)
{

//    write_int(mem, CONTROL_ACTION, 12);
//    write_int(mem, CONTROL_MOVING, 1);
//    std::cout << " | \e[38;5;45mPasse forte Esquerda\e[0m" << std::endl;
//    move_action(1, 0, stop_gait);
//    Action::GetInstance()->Start(70);
//    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
//    Action::GetInstance()->Stop();
//    Action::GetInstance()->m_Joint.SetEnableBody(false);
//    MotionManager::GetInstance()->SetEnable(false);

//    //getchar();

//    // Velocidades
//    /*
//    cm730->WriteWord(7, 32, 255, &erro);
//    cm730->WriteWord(9, 32, 1023, &erro);
//    */
//    cm730->write4ByteTxRx(portHandler, 7, 24, uint32_t(255), &dxl_error);
//    cm730->write4ByteTxRx(portHandler, 9, 24, uint32_t(1023), &dxl_error);

//    /*
//    cm730->WriteWord(7, 30, MotionManager::GetInstance()->m_Offset[7]+603, &erro);
//    cm730->WriteWord(9, 30, MotionManager::GetInstance()->m_Offset[9]+385, &erro);
//    */
//    cm730->write4ByteTxRx(portHandler, 7, 116, uint32_t(MotionManager::GetInstance()->m_Offset[7]+603), &dxl_error);
//    cm730->write4ByteTxRx(portHandler, 9, 116, uint32_t(MotionManager::GetInstance()->m_Offset[9]+385), &dxl_error);

//    //Esperando  completar o movimento
//    unsigned int count_s = 0;
//    uint8_t value = 0;

//    //cm730->ReadWord(9, 46, &value, 0);
//    cm730->read1ByteTxRx(portHandler, 9, 122, &value, &dxl_error);
//    while(value!=0)
//    {
//        count_s++;
//        //cm730->ReadWord(9, 46, &value, 0);
//        cm730->read1ByteTxRx(portHandler, 9, 122, &value, &dxl_error);
//        usleep(8*1000);
//        if(count_s>100)
//            break; //Evita de ficar parado neste laco
//    }
//    //std::cout<<count_s<<std::endl;

//    Action::GetInstance()->m_Joint.SetEnableBody(true);
//    MotionManager::GetInstance()->SetEnable(true);
//    Action::GetInstance()->Start(72);
//    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
//    stop_gait = 1;
//    write_int(mem, CONTROL_MOVING, 0);

}

//==============================================================================
void ActionMove::pass_right(webots::Robot *robot, bool &stop_gait)
{
//    write_int(mem, CONTROL_ACTION, 13);
//    write_int(mem, CONTROL_MOVING, 1);
//    std::cout << " | \e[38;5;45mPasse forte Direita\e[0m" << std::endl;
//    move_action(1, 0, stop_gait);
//    Action::GetInstance()->Start(71);
//    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
//    Action::GetInstance()->Stop();
//    Action::GetInstance()->m_Joint.SetEnableBody(false);
//    MotionManager::GetInstance()->SetEnable(false);

//    //getchar();

//    // Velocidades
//    /*
//    cm730->WriteWord(8, 32, 255, &erro);
//    cm730->WriteWord(10, 32, 1023, &erro);
//    */
//    cm730->write4ByteTxRx(portHandler, 8, 24, uint32_t(255), &dxl_error);
//    cm730->write4ByteTxRx(portHandler, 10, 24, uint32_t(1023), &dxl_error);

//    /*
//    cm730->WriteWord(8, 30, MotionManager::GetInstance()->m_Offset[8]+420, &erro);
//    cm730->WriteWord(10, 30, MotionManager::GetInstance()->m_Offset[10]+638, &erro);
//    */
//    cm730->write4ByteTxRx(portHandler, 8, 116, uint32_t(MotionManager::GetInstance()->m_Offset[8]+420), &dxl_error);
//    cm730->write4ByteTxRx(portHandler, 10, 116, uint32_t(MotionManager::GetInstance()->m_Offset[10]+638), &dxl_error);

//    //Esperando  completar o movimento
//    unsigned int count_s = 0;
//    uint8_t value = 0;

//    //cm730->ReadWord(10, 46, &value, 0);
//    cm730->read1ByteTxRx(portHandler, 10, 122, &value, &dxl_error);
//    while(value!=0)
//    {
//        count_s++;
//        //cm730->ReadWord(10, 46, &value, 0);
//        cm730->read1ByteTxRx(portHandler, 10, 122, &value, &dxl_error);
//        usleep(8*1000);
//        if(count_s>100)
//            break; //Evita de ficar parado neste laco
//    }
//    //std::cout<<count_s<<std::endl;

//    Action::GetInstance()->m_Joint.SetEnableBody(true);
//    MotionManager::GetInstance()->SetEnable(true);
//    Action::GetInstance()->Start(73);
//    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
//    stop_gait = 1;
//    write_int(mem, CONTROL_MOVING, 0);

}

//========================================================================
//Execute the move action-------------------------------------------------
void ActionMove::goalkeeper(bool &stop_gait)
{
    write_int(mem, CONTROL_ACTION, 10);
    write_int(mem, CONTROL_MOVING, 1);
    std::cout<<" | \e[38;5;45mDefender a bola\e[0m"<<std::endl;  //------------------------------TODO
    move_action(1, 0, stop_gait);    /* Init(stand up) pose */
    move_action(20, 0, stop_gait);    // colocar o action-script para cair e defender!!!
    write_int(mem, CONTROL_MOVING, 0);
}

//========================================================================
//Chute fraco direito-----------------------------------------------------
void ActionMove::kick_right_weak(bool &stop_gait)
{
    write_int(mem, CONTROL_ACTION, 21);
    write_int(mem, CONTROL_MOVING, 1);
    std::cout<<" | \e[38;5;45mChute fraco direito\e[0m"<<std::endl;
    move_action(12, 0, stop_gait); 
    write_int(mem, CONTROL_MOVING, 0);
}

//========================================================================
//Chute fraco esquerdo----------------------------------------------------
void ActionMove::kick_left_weak(bool &stop_gait)
{
    write_int(mem, CONTROL_ACTION, 22);
    write_int(mem, CONTROL_MOVING, 1);
    std::cout<<" | \e[38;5;45mChute fraco esquerdo\e[0m"<<std::endl;
    move_action(13, 0, stop_gait);
    write_int(mem, CONTROL_MOVING, 0);
}


//==============================================================================
void ActionMove::kick_left_strong(webots::Robot *robot, bool &stop_gait)
{
//    write_int(mem, CONTROL_ACTION, 5);
//    write_int(mem, CONTROL_MOVING, 1);
//    std::cout << " | \e[38;5;45mChute forte esquerdo\e[0m" << std::endl;
//    move_action(1, 0, stop_gait);
//    Action::GetInstance()->Start(62);
//    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
//    Action::GetInstance()->Stop();
//    Action::GetInstance()->m_Joint.SetEnableBody(false);
//    MotionManager::GetInstance()->SetEnable(false);

//    //getchar();

//    // Velocidades
//    /*
//    cm730->WriteWord(12, 32, 985, &erro);
//    cm730->WriteWord(14, 32, 985, &erro);
//    cm730->WriteWord(16, 32, 1023, &erro);
//    cm730->WriteWord(18, 32, 150, &erro);
//    */
//    cm730->write4ByteTxRx(portHandler, 12, 112, uint32_t(2047), &dxl_error);
//    cm730->write4ByteTxRx(portHandler, 14, 112, uint32_t(1656), &dxl_error);
//    cm730->write4ByteTxRx(portHandler, 16, 112, uint32_t(2032), &dxl_error);
//    cm730->write4ByteTxRx(portHandler, 18, 112, uint32_t(1726), &dxl_error);

//    /*
//    cm730->WriteWord(12, 30, MotionManager::GetInstance()->m_Offset[12]+790, &erro);
//    cm730->WriteWord(14, 30, MotionManager::GetInstance()->m_Offset[14]+600, &erro);
//    cm730->WriteWord(16, 30, MotionManager::GetInstance()->m_Offset[16]+350, &erro);
//    cm730->WriteWord(18, 30, MotionManager::GetInstance()->m_Offset[18]+545, &erro);
//    */

//    cm730->write4ByteTxRx(portHandler, 12, 116, uint32_t(MotionManager::GetInstance()->m_Offset[12]+2470), &dxl_error);
//    cm730->write4ByteTxRx(portHandler, 14, 116, uint32_t(MotionManager::GetInstance()->m_Offset[14]+1999), &dxl_error);
//    cm730->write4ByteTxRx(portHandler, 16, 116, uint32_t(MotionManager::GetInstance()->m_Offset[16]+2453), &dxl_error);
//    cm730->write4ByteTxRx(portHandler, 18, 116, uint32_t(MotionManager::GetInstance()->m_Offset[18]+2085), &dxl_error);


//        //Esperando  completar o movimento
//    unsigned int count_s = 0;
//    uint8_t value = 0;
//     usleep(20*1000);

//    //cm730->ReadWord(16, 46, &value, 0);
//    cm730->read1ByteTxRx(portHandler, 16, 122, &value, &dxl_error);
//    while(value!=0)
//    {
//        count_s++;
//        //cm730->ReadWord(16, 46, &value, 0);
//        cm730->read1ByteTxRx(portHandler, 16, 122, &value, &dxl_error);
//        usleep(8*1000);
//        if(count_s>500)
//            break; //Evita de ficar parado neste laco
//    }
//    //std::cout<<count_s<<std::endl;

//    Action::GetInstance()->m_Joint.SetEnableBody(true);
//    MotionManager::GetInstance()->SetEnable(true);
//    Action::GetInstance()->Start(63);
//    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
//    stop_gait = 1;
//    write_int(mem, CONTROL_MOVING, 0);
}

//==============================================================================
void ActionMove::kick_right_strong(webots::Robot *robot, bool &stop_gait)
{
    write_int(mem, CONTROL_ACTION, 4);
//    write_int(mem, CONTROL_MOVING, 1);
//    std::cout << " | \e[38;5;45mChute forte direito\e[0m" << std::endl;
//    move_action(1, 0, stop_gait);
//    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
//    Action::GetInstance()->Start(60);
//    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
//    Action::GetInstance()->Stop();
//    Action::GetInstance()->m_Joint.SetEnableBody(false);
//    MotionManager::GetInstance()->SetEnable(false);

//    //getchar();

//    // Velocidades
//    /*
//    cm730->WriteWord(11, 32, 1000, &erro);
//    cm730->WriteWord(13, 32, 1000, &erro);
//    cm730->WriteWord(15, 32, 1023, &erro);
//    cm730->WriteWord(17, 32, 97, &erro);
//    */
//    cm730->write4ByteTxRx(portHandler, 11, 112, uint32_t(571), &dxl_error);//292
//    cm730->write4ByteTxRx(portHandler, 13, 112, uint32_t(1023), &dxl_error);//963
//    cm730->write4ByteTxRx(portHandler, 15, 112, uint32_t(760), &dxl_error);//1068
//    cm730->write4ByteTxRx(portHandler, 17, 112, uint32_t(813), &dxl_error);//50

//    cm730->write4ByteTxRx(portHandler, 11, 116, uint32_t(MotionManager::GetInstance()->m_Offset[11]+1353), &dxl_error);
//    cm730->write4ByteTxRx(portHandler, 13, 116, uint32_t(MotionManager::GetInstance()->m_Offset[13]+2424), &dxl_error);
//    cm730->write4ByteTxRx(portHandler, 15, 116, uint32_t(MotionManager::GetInstance()->m_Offset[15]+1800), &dxl_error);
//    cm730->write4ByteTxRx(portHandler, 17, 116, uint32_t(MotionManager::GetInstance()->m_Offset[17]+1927), &dxl_error);

//        //Esperando  completar o movimento
//    unsigned int count_s = 0;
//    uint8_t value = 0;
//    usleep(20*1000);

//    cm730->read1ByteTxRx(portHandler, 15, 122, &value, &dxl_error);
//    while(value!=0)
//    {
//        count_s++;
//        cm730->read1ByteTxRx(portHandler, 15, 122, &value, &dxl_error);
//        usleep(8*1000);
//        if(count_s>500)
//            break; //Evita de ficar parado neste laco
//    }
//    //std::cout<<count_s<<std::endl;

//    Action::GetInstance()->m_Joint.SetEnableBody(true);
//    MotionManager::GetInstance()->SetEnable(true);
//    Action::GetInstance()->Start(61);
//    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
//    stop_gait = 1;
//    write_int(mem, CONTROL_MOVING, 0);

}


//========================================================================
//O robo da acena com a mao-----------------------------------------------
void ActionMove::goodBye(bool &stop_gait)
{
    write_int(mem, CONTROL_ACTION, 20);
    write_int(mem, CONTROL_MOVING, 1);
    std::cout<<" | \e[38;5;45mGoodBye\e[0m"<<std::endl;
    move_action(8, 0, stop_gait);
    write_int(mem, CONTROL_MOVING, 0);
}


//========================================================================
//O robo bate palmas com as maos para cima--------------------------------
void ActionMove::greetings(bool &stop_gait)
{
    write_int(mem, CONTROL_ACTION, 19);
    write_int(mem, CONTROL_MOVING, 1);
    std::cout<<" | \e[38;5;45mGreetings\e[0m"<<std::endl;
    move_action(24, 0, stop_gait); 
    write_int(mem, CONTROL_MOVING, 0);
}

//========================================================================
//Levantar de frente------------------------------------------------------
void ActionMove::standupFront(bool &stop_gait)
{
    write_int(mem, CONTROL_ACTION, 15);
    write_int(mem, CONTROL_MOVING, 1);
    std::cout<<" | \e[38;5;45mLevantar de frente\e[0m"<<std::endl;
    move_action(10, 0, stop_gait);
    write_int(mem, CONTROL_MOVING, 0);
}

//========================================================================
//Levantar de costas------------------------------------------------------
void ActionMove::standupBack(bool &stop_gait)
{
    write_int(mem, CONTROL_ACTION, 16);
    write_int(mem, CONTROL_MOVING, 1);
    std::cout<<" | \e[38;5;45mLevantar de costa\e[0m"<<std::endl;
    move_action(11, 0, stop_gait);
    write_int(mem, CONTROL_MOVING, 0);
}


//========================================================================
//Stand up pose-----------------------------------------------------------
void ActionMove::poseStandup(bool &stop_gait)
{
    write_int(mem, CONTROL_ACTION, 0);
    write_int(mem, CONTROL_MOVING, 1);
    std::cout<<"Start Action 1"<<std::endl;
    move_action(1, 0, stop_gait); 
    write_int(mem, CONTROL_MOVING, 0);
    std::cout<<"Feito"<<std::endl;
    
}


//*********************************************************************
//---------------------------------------------------------------------

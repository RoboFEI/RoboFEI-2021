/*--------------------------------------------------------------------

******************************************************************************
* @file MotionManager.cpp
* @author ROBOTIS
* @version V2.0.0 - ROBOFEI-HT
* @created -
* @Modified Isaac Jesus da Silva - University Center FEI - 27/09/2016  😛
* @e-mail isaac25silva@yahoo.com.br
* @brief MotionManager 😛
****************************************************************************
****************************************************************************
---------------------------------------------------------------------------*/
#include <iomanip>
#include <stdio.h>
#include <math.h>
#include "FSR.h"
#include "MX28.h"
#include "MotionManager.h"
#include <unistd.h>
#include <assert.h>
#include <stdlib.h>
#include <fstream>
#include <time.h>
#include <blackboard.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#define timeStep 8

//#include "dynamixel_sdk.h" //biblioteca SDK

using namespace Robot;

// Torque adaption every second
const int TORQUE_ADAPTION_CYCLES = 1000 / MotionModule::TIME_UNIT;
const int DEST_TORQUE = 2047;

//#define LOG_VOLTAGES 1

MotionManager* MotionManager::m_UniqueInstance = new MotionManager();

MotionManager::MotionManager() :
        //m_CM730(0),
        m_ProcessEnable(false),
        m_Enabled(false),
        m_IsRunning(false),
        m_IsThreadRunning(false),
        m_IsLogging(false),
				m_torqueAdaptionCounter(TORQUE_ADAPTION_CYCLES),
				m_voltageAdaptionFactor(1.0),
        DEBUG_PRINT(false)
{
    for(int i = 0; i < JointData::NUMBER_OF_JOINTS; i++)
        m_Offset[i] = 0;

#if LOG_VOLTAGES
    assert((m_voltageLog = fopen("voltage.log", "w")));
    fprintf(m_voltageLog, "Voltage   Torque\n");
#endif
}

MotionManager::~MotionManager()
{
}

bool MotionManager::Initialize(webots::Robot *robot, bool fadeIn)
{

	uint32_t value;
  dxl_error = 0;
	usleep(10000);
  //this->portHandler = portHandler;
	//m_CM730 = packetHandler;
	this->robot = robot;
	m_Enabled = false;
	m_ProcessEnable = true;
	this->inicio = true;
	
	ShoulderL = robot->getMotor("LeftShoulderRoll [shoulder]");
  ShoulderR = robot->getMotor("RightShoulderRoll [shoulder]");
  ArmUpperL = robot->getMotor("LeftArmPitch [arm]");
  ArmUpperR = robot->getMotor("RightArmPitch [arm]");
  ArmLowerL = robot->getMotor("LeftElbowPitch [arm]");
  ArmLowerR = robot->getMotor("RightElbowPitch [arm]");
  PelvYL = robot->getMotor("LeftHipYaw [hip]");
  PelvYR = robot->getMotor("RightHipYaw [hip]");
  PelvL = robot->getMotor("LeftHipRoll [hip]");
  PelvR = robot->getMotor("RightHipRoll [hip]");
  LegUpperL = robot->getMotor("LeftLegPitch [leg]");
  LegUpperR = robot->getMotor("RightLegPitch [leg]");
  LegLowerL = robot->getMotor("LeftKnee [leg]");
  LegLowerR = robot->getMotor("RightKnee [leg]");
  AnkleL = robot->getMotor("LeftFootPitch [foot]");
  AnkleR = robot->getMotor("RightFootPitch [foot]");
  FootL = robot->getMotor("LeftFootRoll [foot]");
  FootR = robot->getMotor("RightFootRoll [foot]");

//Modo de Operação dos motores para controlar a posição e torque(atraves da corrente).
//  m_CM730->write1ByteTxRx(portHandler, BROADCAST_ID, MX28::P_OPERATING_MODE, 5, &dxl_error);

////Declarando os valores de PID dos bra�os para "Tremer" menos
//  for(int l = 1; l <=8 ;l++)
//  {
//		m_CM730->write2ByteTxRx(portHandler, l, MX28::P_POSITION_D_GAIN, 1000, &dxl_error);
//    m_CM730->write2ByteTxRx(portHandler, l, MX28::P_POSITION_I_GAIN, 50, &dxl_error);
//		m_CM730->write2ByteTxRx(portHandler, l, MX28::P_POSITION_P_GAIN, 400, &dxl_error);
//		m_CM730->write2ByteTxRx(portHandler, l, MX28::P_GOAL_PWM, 600, &dxl_error);
//  }
//  for(int l = 9; l <=10 ;l++)
//  {
//		m_CM730->write2ByteTxRx(portHandler, l, MX28::P_POSITION_D_GAIN, 8000, &dxl_error);
//    m_CM730->write2ByteTxRx(portHandler, l, MX28::P_POSITION_I_GAIN, 0, &dxl_error);
//		m_CM730->write2ByteTxRx(portHandler, l, MX28::P_POSITION_P_GAIN, 5000, &dxl_error);
//		//m_CM730->write2ByteTxRx(portHandler, l, MX28::P_GOAL_PWM, 600, &dxl_error);
//  }

//  for(int l = 11; l <=18 ;l++)
//  {
//    m_CM730->write2ByteTxRx(portHandler, l, MX28::P_POSITION_D_GAIN, 1000, &dxl_error);
//    m_CM730->write2ByteTxRx(portHandler, l, MX28::P_POSITION_I_GAIN, 0, &dxl_error);
//    m_CM730->write2ByteTxRx(portHandler, l, MX28::P_POSITION_P_GAIN, 1000, &dxl_error);
//    //m_CM730->write2ByteTxRx(portHandler, l, MX28::P_GOAL_PWM, 850, &dxl_error);
//  }

////Os motores não ligam se não der o Torque Enable.
//  m_CM730->write1ByteTxRx(portHandler, BROADCAST_ID, MX28::P_TORQUE_ENABLE, 1, &dxl_error);

////When the absolute value of Present Velocity(128) is greater than the Moving Threshold(24), Moving(122) is set to ‘1’, otherwise it is cleared to ‘0’.
//  m_CM730->write4ByteTxRx(portHandler, BROADCAST_ID, MX28::P_MOVING_THRESHOULD, 20, &dxl_error);

////Declarando o valor limite de corrente dos motores, no caso 2047 é o máx.
//  m_CM730->write2ByteTxRx(portHandler, BROADCAST_ID, MX28::P_CURRENT_LIMIT, 2047, &dxl_error);

////Valor máximo do torque dos motores MX-64 é 1941=================================
//  m_CM730->write2ByteTxRx(portHandler, 3, MX28::P_CURRENT_LIMIT, 1941, &dxl_error);
//  m_CM730->write2ByteTxRx(portHandler, 4, MX28::P_CURRENT_LIMIT, 1941, &dxl_error);
//  m_CM730->write2ByteTxRx(portHandler, 5, MX28::P_CURRENT_LIMIT, 1941, &dxl_error);
//  m_CM730->write2ByteTxRx(portHandler, 6, MX28::P_CURRENT_LIMIT, 1941, &dxl_error);

////Valor do Goal Current para os motores iniciarem o código "quase" soltos, se não pode dar o tranco.
//  m_CM730->write2ByteTxRx(portHandler, BROADCAST_ID, MX28::P_GOAL_CURRENT, 2, &dxl_error);

////Declarando o valor limite da velocidade usado pelo P_PROFILE_VELOCITY.
//  m_CM730->write4ByteTxRx(portHandler, BROADCAST_ID, MX28::P_VELOCITY_LIMIT, 1023, &dxl_error);

////Declarando o valor limite da aceleração usado pelo P_PROFILE_ACCELERATION.
//  m_CM730->write4ByteTxRx(portHandler, BROADCAST_ID, MX28::P_ACCELERATION_LIMIT, 32767, &dxl_error);


/*TESTE DA VELOCIDADE DO SERVO
  while(1){
  m_CM730->write2ByteTxRx(portHandler, BROADCAST_ID, MX28::P_GOAL_CURRENT, 2047, &dxl_error);

  m_CM730->write4ByteTxRx(portHandler, BROADCAST_ID, MX28::P_PROFILE_VELOCITY, 50, &dxl_error);

  m_CM730->write4ByteTxRx(portHandler, 7, MX28::P_GOAL_POSITION, 2048, &dxl_error);
}
*/
/*
	if(m_CM730->Connect() == false)
	{
		if(DEBUG_PRINT == true)
			fprintf(stderr, "Fail to connect CM-730\n");
		return false;
	}
*/
//	for(int id=JointData::ID_MIN; id<=JointData::ID_MAX; id++) //diminui tirando a cabeça
//	{
//		if(DEBUG_PRINT == true)
//			fprintf(stderr, "ID:%d initializing...", id);


//		if(m_CM730->read4ByteTxRx(portHandler, id, MX28::P_PRESENT_POSITION, &value, &dxl_error) == COMM_SUCCESS)
//		{
//			MotionStatus::m_CurrentJoints.SetValue(id, value);
//			MotionStatus::m_CurrentJoints.SetEnable(id, true);

//			if(DEBUG_PRINT == true)
//				fprintf(stderr, "[%d] Success\n", value);
//		}
//		else
//		{
//			MotionStatus::m_CurrentJoints.SetEnable(id, false);

//			if(DEBUG_PRINT == true)
//				fprintf(stderr, " Fail\n");
//		}
//	}

	if(fadeIn)
	{
		//for(int i=JointData::ID_R_SHOULDER_PITCH; i<JointData::NUMBER_OF_JOINTS; i++)
			//cm730->WriteWord(i, MX28::P_TORQUE_LIMIT_L, 0, 0);
	}

	m_fadeIn = fadeIn;
	m_torque_count = 0;

	m_CalibrationStatus = 0;
	m_FBGyroCenter = 512;
	m_RLGyroCenter = 512;

	return true;
}

bool MotionManager::Reinitialize()
{
	m_ProcessEnable = false;

	//m_CM730->DXLPowerOn();
	uint32_t value;
  dxl_error = 0;

//	for(int id=JointData::ID_MIN; id<=JointData::ID_MAX; id++)//tirando a cabeça
//	{
//		if(DEBUG_PRINT == true)
//			fprintf(stderr, "ID:%d initializing...", id);

//		if(m_CM730->read4ByteTxRx(portHandler, id, MX28::P_PRESENT_POSITION, &value, &dxl_error) == COMM_SUCCESS)
//		{
//			MotionStatus::m_CurrentJoints.SetValue(id, value);
//			MotionStatus::m_CurrentJoints.SetEnable(id, true);

//			if(DEBUG_PRINT == true)
//				fprintf(stderr, "[%d] Success\n", value);
//		}
//		else
//		{
//			MotionStatus::m_CurrentJoints.SetEnable(id, false);

//			if(DEBUG_PRINT == true)
//				fprintf(stderr, " Fail\n");
//		}
//	}

	m_ProcessEnable = true;
	return true;
}

void MotionManager::Restartrobot()
{
	m_torque_count=0;
}

void MotionManager::StartLogging()
{
    char szFile[32] = {0,};

    int count = 0;
    while(1)
    {
        sprintf(szFile, "Logs/Log%d.csv", count);
        if(0 != access(szFile, F_OK))
            break;
        count++;
		if(count > 256) return;
    }
		m_LogFileStream.open(szFile, std::ios::out);
    for(int id = JointData::ID_MIN; id <= JointData::ID_MAX; id++)//tirando a cabeça
        m_LogFileStream << "nID_" << id << "_GP,nID_" << id << "_PP,";
    m_LogFileStream << "GyroFB,GyroRL,AccelFB,AccelRL,L_FSR_X,L_FSR_Y,R_FSR_X,R_FSR_Y," << "\x0d\x0a";

    m_IsLogging = true;
}

void MotionManager::StopLogging()
{
    m_IsLogging = false;
    m_LogFileStream.close();
}

void MotionManager::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, OFFSET_SECTION);
}
void MotionManager::LoadINISettings(minIni* ini, const std::string &section)
{
    int ivalue = INVALID_VALUE;

    for(int i = JointData::ID_MIN; i <= JointData::ID_MAX; i++)//tirando a cabeça
    {
        char key[10];
        sprintf(key, "ID_%.2d", i);
        if((ivalue = ini->geti(section, key, INVALID_VALUE)) != INVALID_VALUE)  m_Offset[i] = ivalue;
    }
		m_angleEstimator.LoadINISettings(ini, section + "_angle");
}
void MotionManager::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, OFFSET_SECTION);
}
void MotionManager::SaveINISettings(minIni* ini, const std::string &section)
{
    for(int i = JointData::ID_MIN; i <= JointData::ID_MAX; i++)//tirando a cabeça
    {
        char key[10];
        sprintf(key, "ID_%.2d", i);
        ini->put(section, key, m_Offset[i]);
    }
		m_angleEstimator.SaveINISettings(ini, section + "_angle");
}

#define GYRO_WINDOW_SIZE    100
#define ACCEL_WINDOW_SIZE   30
#define MARGIN_OF_SD        2.0
void MotionManager::Process()
{

    //printf( "\n===== Iniciando o Process =====\n\n");
//    if(this->inicio == true){
//        for(int x; x<1000; x++){
//            robot->step(timeStep);
//        }
//        this->inicio = false;
//    }
//    if(m_fadeIn && m_torque_count < DEST_TORQUE) {
//      //printf("entrou\n");
//        if(m_torque_count < 100)
//            m_torque_count += 3;
//        else
//            m_torque_count = 2047 ;

//        //m_CM730->WriteWord(CM730::ID_BROADCAST, MX28::P_TORQUE_LIMIT_L, m_torque_count, 0);
//        m_CM730->write2ByteTxRx(portHandler, BROADCAST_ID, MX28::P_GOAL_CURRENT, m_torque_count, &dxl_error);

//        if(m_torque_count == 2047)
//        {
//            m_CM730->write2ByteTxRx(portHandler, 3, MX28::P_GOAL_CURRENT, 1941, &dxl_error);
//            m_CM730->write2ByteTxRx(portHandler, 4, MX28::P_GOAL_CURRENT, 1941, &dxl_error);
//            m_CM730->write2ByteTxRx(portHandler, 5, MX28::P_GOAL_CURRENT, 1941, &dxl_error);
//            m_CM730->write2ByteTxRx(portHandler, 6, MX28::P_GOAL_CURRENT, 1941, &dxl_error);
//        }

///*TESTE PARA LIGAR O SERVO
//        m_CM730->write4ByteTxRx(portHandler, 7, MX28::P_GOAL_POSITION, 2048, &dxl_error);

//        uint16_t voltage;
//        uint16_t current;

//        m_CM730->read2ByteTxRx(portHandler, 7, MX28::P_PRESENT_VOLTAGE, &voltage, &dxl_error);
//        m_CM730->read2ByteTxRx(portHandler, 7, MX28::P_PRESENT_CURRENT, &current, &dxl_error);

//        std::cout<<"m_torque_count "<<int (m_torque_count)<<std::endl;
//        std::cout<<"tenso "<<voltage<<std::endl;
//        std::cout<<"current "<<current<<std::endl;
//*/
//    }

    if(m_ProcessEnable == false || m_IsRunning == true)
        return;

		m_IsRunning = true;


        m_CalibrationStatus = 1;


    if(m_CalibrationStatus == 1 && m_Enabled == true)
    {
    
    

          const double GYRO_ALPHA = 0.1;
          int gyroValFB = (int) (-read_int(mem, IMU_GYRO_Y)*16);
          int gyroValRL = (int) (read_int(mem, IMU_GYRO_X)*16);


//          const double GYRO_ALPHA = 0.1;
//          int gyroValFB = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L) - m_FBGyroCenter;
//          int gyroValRL = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L) - m_RLGyroCenter;

//          MotionStatus::FB_GYRO = (1.0 - GYRO_ALPHA) * MotionStatus::FB_GYRO + GYRO_ALPHA * gyroValFB;
//          MotionStatus::RL_GYRO = (1.0 - GYRO_ALPHA) * MotionStatus::RL_GYRO + GYRO_ALPHA * gyroValRL;;
//          MotionStatus::RL_ACCEL = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L);
//          MotionStatus::FB_ACCEL = m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L);


          MotionStatus::FB_GYRO = (1.0 - GYRO_ALPHA) * MotionStatus::FB_GYRO + GYRO_ALPHA * gyroValFB;
          MotionStatus::RL_GYRO = (1.0 - GYRO_ALPHA) * MotionStatus::RL_GYRO + GYRO_ALPHA * gyroValRL;


//           m_angleEstimator.update(
//               (m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L) - 512),
//               (m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L) - 512),
//               (m_CM730->m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Z_L) - 512)
//           );

//           MotionStatus::ANGLE_PITCH = m_angleEstimator.pitch();
//           MotionStatus::ANGLE_ROLL  = m_angleEstimator.roll();
//					}

//        int sum = 0, avr//printf("entrou\n"); = 512;
//        for(int idx = 0; idx < ACCEL_WINDOW_SIZE; idx++)
//            sum += fb_array[idx];
//        avr = sum / ACCEL_WINDOW_SIZE;

//        if(avr < MotionStatus::FALLEN_F_LIMIT)
//            MotionStatus::FALLEN = FORWARD;
//        else if(avr > MotionStatus::FALLEN_B_LIMIT)
//            MotionStatus::FALLEN = BACKWARD;
//        else
//            MotionStatus::FALLEN = STANDUP;



 	if(m_Modules.size() != 0)
	{
		for(std::list<MotionModule*>::iterator i = m_Modules.begin(); i != m_Modules.end(); i++)
		{
			(*i)->Process();
			for(int id=JointData::ID_MIN; id<=JointData::ID_MAX; id++)
			{
				if((*i)->m_Joint.GetEnable(id) == true)
				{
//				MotionStatus::m_CurrentJoints.SetSlope(id, (*i)->m_Joint.GetCWSlope(id), (*i)->m_Joint.GetCCWSlope(id));
         MotionStatus::m_CurrentJoints.SetValue(id, (*i)->m_Joint.GetValue(id));
				 MotionStatus::m_CurrentJoints.SetPGain(id, (*i)->m_Joint.GetPGain(id));
				 MotionStatus::m_CurrentJoints.SetIGain(id, (*i)->m_Joint.GetIGain(id));
				 MotionStatus::m_CurrentJoints.SetDGain(id, (*i)->m_Joint.GetDGain(id));

				}
			}
		}

// 	if(m_Modules.size() != 0)
//	{
//		for(std::list<MotionModule*>::iterator i = m_Modules.begin(); i != m_Modules.end(); i++)
//		{
//			(*i)->Process();
//			for(int id=JointData::ID_MIN; id<=JointData::ID_MAX; id++)
//			{
//				if((*i)->m_Joint.GetEnable(id) == true)
//				{
////				MotionStatus::m_CurrentJoints.SetSlope(id, (*i)->m_Joint.GetCWSlope(id), (*i)->m_Joint.GetCCWSlope(id));
//         MotionStatus::m_CurrentJoints.SetValue(id, (*i)->m_Joint.GetValue(id));
//				 MotionStatus::m_CurrentJoints.SetPGain(id, (*i)->m_Joint.GetPGain(id));
//				 MotionStatus::m_CurrentJoints.SetIGain(id, (*i)->m_Joint.GetIGain(id));
//				 MotionStatus::m_CurrentJoints.SetDGain(id, (*i)->m_Joint.GetDGain(id));

//				}
//			}
//		}
	}
	//uint8_t param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];

  //param = new uint8_t[21 * (1 + 5)];

    int param [JointData::NUMBER_OF_JOINTS * (1 + MX28::PARAM_BYTES)]; // ID(1) + DATA(data_length)
	int n = 0;
	uint16_t joint_num = 0;
  uint32_t present;

  uint8_t wGoalPosition = 100;
  uint8_t wDistance = 200;
  

	for(uint8_t id=JointData::ID_MIN; id<=JointData::ID_MAX; id++)
	{
		if(MotionStatus::m_CurrentJoints.GetEnable(id) == true)
		{
                //param[n++] = id;

                //TESTE SE BASEANDO NO ACTION EDITOR - FUNCIONA!!
/*
                param[n++] = DXL_LOBYTE(DXL_LOWORD(wGoalPosition));
                param[n++] = DXL_HIBYTE(DXL_LOWORD(wGoalPosition));
                param[n++] = DXL_LOBYTE(DXL_HIWORD(wDistance));
                param[n++] = DXL_HIBYTE(DXL_HIWORD(wDistance));
*/
              /*
#ifdef MX28_1024
                param[n++] = MotionStatus::m_CurrentJoints.GetCWSlope(id);
                param[n++] = MotionStatus::m_CurrentJoints.GetCCWSlope(id);
#else

                param[n++] = MotionStatus::m_CurrentJoints.GetDGain(id);
                param[n++] = MotionStatus::m_CurrentJoints.GetIGain(id);
                int p_gain = m_voltageAdaptionFactor * MotionStatus::m_CurrentJoints.GetPGain(id);
                if(p_gain <= 0)
                    p_gain = 1;
                param[n++] = p_gain;
                param[n++] = 0;

#endif*
                param[n++] = CM730::GetLowByte(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id]);
                param[n++] = CM730::GetHighByte(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id]);
*/

                param[id] = ((MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id]) - 2047);
                //printf("p[%d] = %d\n", id, param[id]);
                //printf("p[%d] = %.7f\n", id, param[id]*0.001533203125);

 //TESTE DA DYNAMIXEL - FUNCIONA COM PARAM_BYTES = 4
//                param[n++] = (DXL_LOBYTE(DXL_LOWORD(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id])));
//                param[n++] = (DXL_HIBYTE(DXL_LOWORD(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id])));
//                param[n++] = (DXL_LOBYTE(DXL_HIWORD(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id])));
//                param[n++] = (DXL_HIBYTE(DXL_HIWORD(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id])));

/*
                param[n++] = (DXL_LOBYTE(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id]));
                param[n++] = (DXL_HIBYTE(MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id]));
*/
				joint_num++;


//TESTE COM O PID = UTILIZANDO O PID OS MOTORES NÃO FICAM TRAVADOS NA POSIÇÃO DESEJADA COM A CORRENTE MÁXIMA.
/*
                m_CM730->write2ByteTxRx(portHandler, id, MX28::P_POSITION_D_GAIN, (MotionStatus::m_CurrentJoints.GetDGain(id)), &dxl_error);
                m_CM730->write2ByteTxRx(portHandler, id, MX28::P_POSITION_I_GAIN, (MotionStatus::m_CurrentJoints.GetIGain(id)), &dxl_error);

                int p_gain = uint16_t (m_voltageAdaptionFactor * (MotionStatus::m_CurrentJoints.GetPGain(id)));
                  if(p_gain <= 0)
                    p_gain = uint16_t(1);
                m_CM730->write2ByteTxRx(portHandler, id, MX28::P_POSITION_P_GAIN, p_gain, &dxl_error);
*/
    }

		if(DEBUG_PRINT == true)
		fprintf(stderr, "ID[%d] : %d \n", id, MotionStatus::m_CurrentJoints.GetValue(id));
	}

	if(joint_num > 0){


        //m_CM730->SyncWrite(MX28::P_CW_COMPLIANCE_SLOPE, MX28::PARAM_BYTES, joint_num, param);
        //printf( "\n===== Enviando valores para os motores ============================\n\n");
        //m_CM730->syncWriteTxOnly(portHandler, MX28::P_GOAL_POSITION, MX28::PARAM_BYTES, param, (joint_num  * (1 + MX28::PARAM_BYTES)));
          //while(robot->step(timeStep)){
//          printf("p[7] = %f\n", (param[7]*0.001533203125)); 
//          printf("p[8] = %f\n", (param[8]*0.001533203125));
            ShoulderR->setPosition(param[1]*0.001533203125); //0.018533
            ShoulderL->setPosition(param[2]*0.001533203125);   //0.012265
            ArmUpperR->setPosition(param[3]*0.001533203125); //-0.036158
            ArmUpperL->setPosition(param[4]*0.001533203125);   //0.018398
            ArmLowerR->setPosition(param[5]*0.001533203125); //-1.90
            ArmLowerL->setPosition(param[6]*-0.001533203125);//-1.90
            PelvYR->setPosition(param[7]*0.001533203125);
            PelvYL->setPosition(param[8]*0.001533203125);
            PelvR->setPosition(param[9]*-0.001533203125);
            PelvL->setPosition(param[10]*-0.001533203125);
            LegUpperR->setPosition(param[11]*0.001533203125);
            LegUpperL->setPosition(param[12]*0.001533203125); // -0.00093203125
            LegLowerR->setPosition(param[13]*0.001533203125);
            LegLowerL->setPosition(param[14]*0.001533203125);
            AnkleR->setPosition(param[15]*0.001533203125);    // -0.00180203125
            AnkleL->setPosition(param[16]*0.001533203125);    // -0.00180203125
            FootR->setPosition(param[17]*0.001533203125);
            FootL->setPosition(param[18]*0.001533203125);
            robot->step(timeStep);
            //printf( "\n===== %d ============================\n\n", robot->step(timeStep));
    
    }
/*
    std::cout << "goal 7: " << int (wGoalPosition) << '\n';

    m_CM730->read4ByteTxRx(portHandler, 7, MX28::P_PRESENT_POSITION, &present, &dxl_error);

    std::cout << "present 7: " << present << '\n';
*/
  /*
  #ifdef MX28_1024
            //m_CM730->SyncWrite(MX28::P_CW_COMPLIANCE_SLOPE, MX28::PARAM_BYTES, joint_num, param);
            //m_CM730->syncWriteTxOnly(portHandler, MX28::P_CW_COMPLIANCE_SLOPE, MX28::PARAM_BYTES, joint_num, param);

  #else
            //m_CM730->SyncWrite(MX28::P_POSITION_D_GAIN, MX28::PARAM_BYTES, joint_num, param);
            //m_CM730->syncWriteTxOnly(portHandler, MX28::P_POSITION_D_GAIN, MX28::PARAM_BYTES, param, joint_num  * (1 + MX28::PARAM_BYTES));
            m_CM730->syncWriteTxOnly(portHandler, MX28::P_GOAL_POSITION, MX28::PARAM_BYTES, param, (joint_num  * 6));
  #endif
*/

	}


    m_IsRunning = false;

//    if(m_torque_count != DEST_TORQUE && --m_torqueAdaptionCounter == 0)
//    {
//        m_torqueAdaptionCounter = TORQUE_ADAPTION_CYCLES;
//        adaptTorqueToVoltage();
//    }
}

void MotionManager::SetEnable(bool enable)
{

	//printf("entrou\n");
  uint32_t valor = 1020; //declarado "0" -= velocidade infinita.
	m_Enabled = enable;
	if(m_Enabled == true){}
		//m_CM730->WriteWord(CM730::ID_BROADCAST, MX28::P_MOVING_SPEED_L, 200, 0);
    //m_CM730->write4ByteTxRx(portHandler, BROADCAST_ID, MX28::P_PROFILE_VELOCITY, valor, &dxl_error);
		//m_CM730->WriteWord(1, 30, 900, 0);

}

void MotionManager::AddModule(MotionModule *module)
{
	module->Initialize();
	m_Modules.push_back(module);

}

void MotionManager::RemoveModule(MotionModule *module)
{
	m_Modules.remove(module);
}

void MotionManager::SetJointDisable(int index)
{
    if(m_Modules.size() != 0)
    {
        for(std::list<MotionModule*>::iterator i = m_Modules.begin(); i != m_Modules.end(); i++)
            (*i)->m_Joint.SetEnable(index, false);
    }
}

void MotionManager::adaptTorqueToVoltage()
{
//	static int count_fail=0;
//	static int count_volt=0;
//  const int DEST_TORQUE = 1023;
//  const int FULL_TORQUE_VOLTAGE = 210; // 13V - at 13V darwin will make no adaptation as the standard 3 cell battery is always below this voltage, this implies Nimbro-OP runs on 4 cells

//  uint16_t voltage;
//	// torque is only reduced if it is greater then FULL_TORQUE_VOLTAGE
//	//if(m_CM730->ReadByte(7, 42, &voltage, 0) != CM730::SUCCESS && m_CM730->ReadByte(8, 42, &voltage, 0) != CM730::SUCCESS)
//  if(m_CM730->read2ByteTxRx(portHandler, 7, MX28::P_PRESENT_VOLTAGE, &voltage, &dxl_error) != COMM_SUCCESS && m_CM730->read2ByteTxRx(portHandler, 7, MX28::P_PRESENT_VOLTAGE, &voltage, &dxl_error) != COMM_SUCCESS)
//	{

//    	count_fail++;
//    	if(count_fail>=7)
//    	{
//    		printf("Falha na comunicação: Chave provavelmente desligada\n");
//    		logServo(); //Escreve no arquivo de log a hora que terminou o processo
//    		exit(0);
//    	}
//      return;
//  }
//  count_fail=0;

//  if(voltage < 128)
//  {
//      count_volt++;
//      if(count_volt>=4)
//      {
//		    printf("Tensão Abaixo do recomendado | Tensão = %2.1fV\n", (float)voltage/(float)10);
//		    printf("A bateria deve ser trocada\n");
//		    logVoltage(voltage); //Escreve no arquivo de log a tensão e a hora que terminou o processo
//		    exit(0);
//		  }
//  }
//  else
//      count_volt=0;
//  write_int(mem, VOLTAGE, voltage);

//    //if(m_CM730->ReadByte(200, CM730::P_VOLTAGE, &voltage, 0) != CM730::SUCCESS)
//  return;

//    voltage = (voltage > FULL_TORQUE_VOLTAGE) ? voltage : FULL_TORQUE_VOLTAGE;
//    m_voltageAdaptionFactor = ((double)FULL_TORQUE_VOLTAGE) / voltage;
//    uint16_t torque = m_voltageAdaptionFactor * DEST_TORQUE;

//#if LOG_VOLTAGES
//    fprintf(m_voltageLog, "%3d       %4d\n", voltage, torque);
//#endif

//    //m_CM730->WriteWord(CM730::ID_BROADCAST, MX28::P_TORQUE_LIMIT_L, torque, 0);
//    m_CM730->write2ByteTxRx(portHandler, BROADCAST_ID, MX28::P_GOAL_CURRENT, torque, &dxl_error);
}

void MotionManager::logVoltage(int voltage)
{
        std::fstream File;
        time_t _tm =time(NULL);
        struct tm * curtime = localtime ( &_tm );
        File.open("../../Control/Control.log", std::ios::app | std::ios::out);
        if (File.good() && File.is_open())
        {
            File << "Tensão Abaixo do recomendado | Tensão = "<<(float)voltage/(float)10<<"V --- ";
            File << asctime(curtime);
            File.flush();
            File.close();
        }
        else
	        printf("Erro ao Salvar o arquivo\n");
}

void MotionManager::logServo()
{
        std::fstream File;
        time_t _tm =time(NULL);
        struct tm * curtime = localtime ( &_tm );
        File.open("../../Control/Control.log", std::ios::app | std::ios::out);
        if (File.good() && File.is_open())
        {
            File << "Falha na comunicação: Chave provavelmente desligada"<<" --- ";
            File << asctime(curtime);
            File.flush();
            File.close();
        }
        else
	        printf("Erro ao Salvar o arquivo\n");
}

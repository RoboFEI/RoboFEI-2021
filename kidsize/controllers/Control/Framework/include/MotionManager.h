/*
 *   MotionManager.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _MOTION_MANGER_H_
#define _MOTION_MANGER_H_


#include <list>
#include <fstream>
#include <iostream>
#include "MotionStatus.h"
#include "MotionModule.h"
#include "minIni.h"
#include "AngleEstimator.h"
#include "../Framework_Dynamixel/include/dynamixel_sdk/dynamixel_sdk.h"
//#include <webots/Robot.hpp>
#include <../../../../../../include/controller/cpp/webots/Robot.hpp>


#define OFFSET_SECTION "Offset"
#define INVALID_VALUE   -1024.0


namespace Robot
{
	class MotionManager
	{
	private:
		static MotionManager* m_UniqueInstance;
		std::list<MotionModule*> m_Modules;
		//dynamixel::PortHandler *portHandler;
		//dynamixel::PacketHandler *m_CM730;
		webots::Robot *robot;
		webots::Motor *ShoulderL;
        webots::Motor *ShoulderR;
        webots::Motor *ArmLowerL;
        webots::Motor *ArmLowerR;
        webots::Motor *ArmUpperL;
        webots::Motor *ArmUpperR;
        webots::Motor *PelvYL;
        webots::Motor *PelvYR;
        webots::Motor *PelvL;
        webots::Motor *PelvR;
        webots::Motor *LegLowerL;
        webots::Motor *LegLowerR;
        webots::Motor *LegUpperL;
        webots::Motor *LegUpperR;
        webots::Motor *AnkleL;
        webots::Motor *AnkleR;
        webots::Motor *FootL;
        webots::Motor *FootR;

		
		bool inicio;
		
		uint8_t dxl_error;
		bool m_ProcessEnable;
		bool m_Enabled;
		int m_FBGyroCenter;
		int m_RLGyroCenter;
		int m_CalibrationStatus;

		bool m_IsRunning;
		bool m_IsThreadRunning;
		bool m_IsLogging;

		std::ofstream m_LogFileStream;

	AngleEstimator m_angleEstimator;
	bool m_fadeIn;
	uint16_t m_torque_count;

	FILE* m_voltageLog;

	unsigned int m_torqueAdaptionCounter;
	double m_voltageAdaptionFactor;

	MotionManager();

	void adaptTorqueToVoltage();

	void logVoltage(int voltage);

	void logServo();

	protected:

	public:
		bool DEBUG_PRINT;
    	int m_Offset[JointData::NUMBER_OF_JOINTS];
    	int *memBB;

		~MotionManager();

		static MotionManager* GetInstance() { return m_UniqueInstance; }

		bool Initialize(webots::Robot *robot, bool fadeIn = true);
		bool Reinitialize();
		void Restartrobot();
        void Process();
		void SetEnable(bool enable);
		bool GetEnable()				{ return m_Enabled; }
		void AddModule(MotionModule *module);
		void RemoveModule(MotionModule *module);

		void ResetGyroCalibration() { m_CalibrationStatus = 0; m_FBGyroCenter = 512; m_RLGyroCenter = 512; }
		int GetCalibrationStatus() { return m_CalibrationStatus; }
		void SetJointDisable(int index);

		void StartLogging();
		void StopLogging();

        void LoadINISettings(minIni* ini);
        void LoadINISettings(minIni* ini, const std::string &section);
        void SaveINISettings(minIni* ini);
        void SaveINISettings(minIni* ini, const std::string &section);

		inline AngleEstimator* angleEstimator()		{ return &m_angleEstimator; }
};
}

#endif

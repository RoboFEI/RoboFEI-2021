#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <iostream>

#define timeStep 8
using namespace webots;
using namespace std;

int main(int argc, char **argv) {

  Robot *robot = new Robot();
  
  Motor *Neck = robot->getMotor("Neck [head]");
  
  Motor *ShoulderL = robot->getMotor("LeftShoulderRoll [shoulder]");
  Motor *ArmUpperL = robot->getMotor("LeftArmPitch [arm]");
  Motor *ArmLowerL = robot->getMotor("LeftElbowPitch [arm]");
  
  Motor *ShoulderR = robot->getMotor("RightShoulderRoll [shoulder]");
  Motor *ArmUpperR = robot->getMotor("RightArmPitch [arm]");
  Motor *ArmLowerR = robot->getMotor("RightElbowPitch [arm]");
  
  Motor *PelvYL = robot->getMotor("LeftHipYaw [hip]");
  Motor *PelvL = robot->getMotor("LeftHipRoll [hip]");
  Motor *LegUpperL = robot->getMotor("LeftLegPitch [leg]");
  Motor *LegLowerL = robot->getMotor("LeftKnee [leg]");
  Motor *AnkleL = robot->getMotor("LeftFootPitch [foot]");
  Motor *FootL = robot->getMotor("LeftFootRoll [foot]");
  
  Motor *PelvYR = robot->getMotor("RightHipYaw [hip]");
  Motor *PelvR = robot->getMotor("RightHipRoll [hip]");
  Motor *LegUpperR = robot->getMotor("RightLegPitch [leg]");
  Motor *LegLowerR = robot->getMotor("RightKnee [leg]");
  Motor *AnkleR = robot->getMotor("RightFootPitch [foot]");
  Motor *FootR = robot->getMotor("RightFootRoll [foot]");

  int t = -1;
  while(robot->step(timeStep) != t){

  Neck->setPosition(0.00);
  ShoulderL->setPosition(0.00);
  ArmUpperL->setPosition(-2.434);
  ArmLowerL->setPosition(0.00);
  ShoulderR->setPosition(0.00);
  ArmUpperR->setPosition(2.434);
  ArmLowerR->setPosition(0.00);
  PelvYL->setPosition(0.00);
  PelvL->setPosition(0.00);
  LegUpperL->setPosition(0.00);
  LegLowerL->setPosition(0.00);
  AnkleL->setPosition(0.00);
  FootL->setPosition(0.00);
  PelvYR->setPosition(0.00);
  PelvR->setPosition(0.00);
  LegUpperR->setPosition(0.00);
  LegLowerR->setPosition(0.00);
  AnkleR->setPosition(0.00);
  FootR->setPosition(0.00);
  t = 0;
  };
  
  t= -1;
  while(robot->step(timeStep) != t){

  Neck->setPosition(0.00);
  ShoulderL->setPosition(0.00);
  ArmUpperL->setPosition(-2.434);
  ArmLowerL->setPosition(0.00);
  ShoulderR->setPosition(0.00);
  ArmUpperR->setPosition(2.434);
  ArmLowerR->setPosition(0.00);
  PelvYL->setPosition(0.00);
  PelvL->setPosition(-0.630);
  LegUpperL->setPosition(0.00);
  LegLowerL->setPosition(0.00);
  AnkleL->setPosition(0.453);
  FootL->setPosition(0.00);
  PelvYR->setPosition(0.00);
  PelvR->setPosition(0.623);
  LegUpperR->setPosition(0.00);
  LegLowerR->setPosition(0.00);
  AnkleR->setPosition(0.453);
  FootR->setPosition(0.00);
  
  }
  
  delete robot;
  return 0;
}
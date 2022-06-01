/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       seankim                                                   */
/*    Created:      Sat Apr 30 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftFront            motor         1               
// RightFront           motor         2               
// LeftBack             motor         9               
// RightBack            motor         10              
// FrontGrabber         motor         5               
// FrontArm             motor         3               
// BackArm              motor         8               
// Intaker              motor         17              
// gyro1                inertial      19              
// ---- END VEXCODE CONFIGURED DEVICES ----


#include "vex.h"

motor_group leftgroup(LeftBack, LeftFront);
motor_group rightgroup(RightBack, RightFront);

smartdrive superdrive(leftgroup, rightgroup, gyro1, 32, 30, 38);

void forw() {
  LeftFront.spin(forward);
  RightFront.spin(forward);
  LeftBack.spin(forward);
  RightBack.spin(forward);
} //move forward

void back() {
  LeftFront.spin(reverse);
  RightFront.spin(reverse);
  LeftBack.spin(reverse);
  RightBack.spin(reverse);
} //move backward

void tright() {
  LeftFront.spin(reverse);
  RightFront.spin(forward);
  LeftBack.spin(reverse);
  RightBack.spin(forward);
} //turn right

void tleft() {
  LeftFront.spin(forward);
  RightFront.spin(reverse);
  LeftBack.spin(forward);
  RightBack.spin(reverse);
} //turn left

void smoothDriveFor(int dist, int vel, int dir){
  float a = 0.1;
  float b = 0.5;

  //if int dir = 0, reverse, dir = 1, forward
  if(dir == 0) {
    superdrive.driveFor(reverse, dist*a, distanceUnits::cm, vel*b, velocityUnits::pct); //acceleration
    superdrive.driveFor(reverse, dist*(1-2*a), distanceUnits::cm, vel, velocityUnits::pct); //full speed
    superdrive.driveFor(reverse, dist*a, distanceUnits::cm, vel*b, velocityUnits::pct); //decelleratoin and stopping any possible drift
  }

  else {
    superdrive.driveFor(forward, dist*a, distanceUnits::cm, vel*b, velocityUnits::pct); //acceleration
    superdrive.driveFor(forward, dist*(1-2*a), distanceUnits::cm, vel, velocityUnits::pct); //full speed
    superdrive.driveFor(forward, dist*a, distanceUnits::cm, vel*b, velocityUnits::pct); //decelleratoin and stopping any possible drift
  }
}


void auton() {
  gyro1.calibrate();
  while(gyro1.isCalibrating()) wait(500, msec);
  gyro1.setHeading(0, degrees); 

  Intaker.setVelocity(100, percent); 
  LeftFront.setVelocity(50, percent); 
  RightFront.setVelocity(50, percent); 
  LeftBack.setVelocity(50, percent); 
  RightBack.setVelocity(50, percent); 
  
  while(BackArm.position(degrees)<=1030) BackArm.spin(forward, 60, velocityUnits::pct);

  smoothDriveFor(65, 40, 0);

  while(BackArm.position(degrees)<=750) BackArm.spin(forward, 60, velocityUnits::pct);

  while(FrontArm.position(degrees)>=-600) BackArm.spin(reverse, 60, velocityUnits::pct);

  Intaker.spinFor(forward, 1800, degrees, false); 
  
  RightFront.spinFor(forward, 10, degrees, false); 

  RightBack.spinFor(forward, 10, degrees, false); 
  
  while(gyro1.rotation(degrees)>=280) tleft();

  smoothDriveFor(40, 40, 0);

  while(gyro1.rotation(degrees)>=190) tleft();

  smoothDriveFor(240, 40, 1);



}

void frontup() {
  FrontArm.spin(reverse, 60, velocityUnits::pct);
  waitUntil(!Controller1.ButtonR1.pressing());
  FrontArm.stop(vex::brakeType::hold);
}
void frontdown() {
  FrontArm.spin(forward, 60, velocityUnits::pct);
  waitUntil(!Controller1.ButtonR2.pressing());
  FrontArm.stop(vex::brakeType::hold);
}
void backup() {
 BackArm.spin(reverse, 60, velocityUnits::pct);
  while(BackArm.position(degrees)>=750) BackArm.spin(reverse, 60, velocityUnits::pct);
  waitUntil(!Controller1.ButtonL1.pressing());
  BackArm.stop(vex::brakeType::hold);
}
void backdown() {
  while(BackArm.position(degrees)<=1030) BackArm.spin(forward, 60, velocityUnits::pct);
  waitUntil(!Controller1.ButtonL2.pressing());
  BackArm.stop(vex::brakeType::hold);
}
void grab() {
  FrontGrabber.spin(forward, 60, velocityUnits::pct);
  waitUntil(!Controller1.ButtonUp.pressing());
  FrontGrabber.stop(vex::brakeType::hold);
}
void release() {
  FrontGrabber.spin(reverse, 60, velocityUnits::pct);
  waitUntil(!Controller1.ButtonDown.pressing());
  FrontGrabber.stop(vex::brakeType::hold);
}

void intake() {
  Intaker.spin(reverse, 1000, velocityUnits::pct);
  waitUntil(!Controller1.ButtonDown.pressing());
  Intaker.stop(vex::brakeType::hold);
}

void in_u(){
  Intaker.spin(forward);
}

void in_S(){
  Intaker.stop();
}


void usercontrol(void) {

  while(true) {
    LeftFront.spin(vex::directionType::rev,Controller1.Axis3.position(),vex::velocityUnits::pct);
    RightFront.spin(vex::directionType::fwd,Controller1.Axis2.position(),vex::velocityUnits::pct);
    LeftBack.spin(vex::directionType::rev,Controller1.Axis3.position(),vex::velocityUnits::pct);
    RightBack.spin(vex::directionType::fwd,Controller1.Axis2.position(),vex::velocityUnits::pct);

    
    if(Controller1.ButtonL1.pressing()){
      backup();
    }
    else if(Controller1.ButtonL2.pressing()){
      backdown();
    }
    else if(Controller1.ButtonR1.pressing()){
      frontup();
    }
    else if(Controller1.ButtonR2.pressing()){
      frontdown();
    }
    else if(Controller1.ButtonUp.pressing()){
      grab();
    }
    else if(Controller1.ButtonDown.pressing()){
      release();
    }
   
    Intaker.setVelocity(100, percent); 

    Controller1.ButtonA.pressed(in_u);
    Controller1.ButtonA.released(in_S);

    

    vex::task::sleep(20);
  }
}

int main() {
  
  // auton();
  while(true) {
    usercontrol();
    vex::task::sleep(100);
  }
  vexcodeInit();
}
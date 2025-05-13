#include "vex.h"
#include "pictures.h"


// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name] [Type] [Port(s)]
// rightFront motor 9
// rightBack motor 15
// rightMid motor 21
// leftFront motor 4
// leftBack motor 14
// leftMid motor 12
// Controller1 controller
// Intake motor 1
// Inertial inertial 5
// MOGO digital_out A
// detected distance 10
// Upper motor 20
// mogo digital_out B
//sensor optical 3
// aligner distance 13
// WallStakes motor 17
// rotationSensor rotation 6
// Doinker digital_out C
// ---- END VEXCODE CONFIGURED DEVICES ----


using namespace vex;
competition Competition;






Drive chassis(


ZERO_TRACKER_ODOM,






//Left Motors:
  motor_group(leftFront,leftMid,leftBack),
//Right Motors:
  motor_group(rightFront, rightBack, rightMid),

PORT20,

3.25, 0.8,

//Gyro scale, this is what inertial reads when bot spins 360 degrees.

360,

//Unused bc no holonomic drive

//LF: //RF:
PORT2, -PORT20,


//LB: //RB:
PORT17, -PORT11,


//JAR ODOM setup


//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
3,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
3.25,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
5.75,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
1,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5


);



//AntiJam code stopper variable

bool codestopper129 = false;




void arcadeDrive(){ 
  
  float throttle = deadband(controller(primary).Axis3.value(), 1);
  float turn = deadband(controller(primary).Axis1.value(), 12);
  leftFront.spin(fwd, to_volt(throttle+turn), percent);
  leftMid.spin(fwd, to_volt(throttle+turn), percent);
  leftBack.spin(fwd, to_volt(throttle+turn), percent);
  rightFront.spin(fwd, to_volt(throttle-turn), percent);
  rightMid.spin(fwd, to_volt(throttle-turn), percent);
  rightBack.spin(fwd, to_volt(throttle-turn), percent);

  }


void ThrowAwayBlue() {

  color blue = color((0, 0, 255));

  if (colorsorter1.color() == blue) {
  Controller1.rumble("...");
  sorter.set(true);
  wait(1000,msec);
  sorter.set(false);
  //Intake.stop(hold);

  }
}



void ThrowAwayRed() {

  colorsorter1.setLightPower(100,percent);

  if (colorsorter1.color() == red) {
  Controller1.rumble("...");
  sorter.set(true);
  wait(1000,msec);
  sorter.set(false);
  //Intake.stop(hold);

  }
}

void driveCurved(double targetPosition, double speed, int curve, double kp) {
  // reset motor positions
  leftFront.resetPosition();
  leftMid.resetPosition();
  leftBack.resetPosition();
  rightFront.resetPosition();
  rightMid.resetPosition();
  rightBack.resetPosition();

  //stop all motors
  leftFront.stop(brake);
  leftMid.stop(brake);
  leftBack.stop(brake);
  rightFront.stop(brake);
  rightMid.stop(brake);
  rightBack.stop(brake);

  double realTarget = (targetPosition * 15); //scales up the target so inputted numbers can be lower
  double error = 0.0;
  double output = 0.0;


  double tolerance = 5.0;
  double speedLimit = (speed / 200);


  while (true) {


  // error
  double currentPosition = (leftFront.position(degrees) + rightFront.position(degrees)) / 2;
  error = realTarget - currentPosition;




  output = kp * error;


  //stops output from overheating motors
  if (output > 100.0) {
  output = 100.0;
  } else if (output < -100.0) {
  output = -100.0;
  }


  // adjust output for curve
  double leftSpeed = output;
  double rightSpeed = output;


  // positive curve = right motors slower
  if (curve > 0) {
  rightSpeed = output - (output * curve / 100);
  }
  // negative curve = left motors slower
  else if (curve < 0) {
  leftSpeed = output - (output * (-curve) / 100);
  }


  // gives max speed
  leftSpeed *= speedLimit;
  rightSpeed *= speedLimit;


  // spins motors
  leftFront.spin(forward, leftSpeed, percent);
  leftMid.spin(forward, leftSpeed, percent);
  leftBack.spin(forward, leftSpeed, percent);
  rightFront.spin(forward, rightSpeed, percent);
  rightMid.spin(forward, rightSpeed, percent);
  rightBack.spin(forward, rightSpeed, percent);


  // checks if error is less than tolerace
  if (fabs(error) < tolerance) {
  // Stop all motors
  leftFront.stop();
  leftMid.stop();
  leftBack.stop(brake);
  rightFront.stop();
  rightMid.stop();
  rightBack.stop(brake);
  wait(20,msec);
  break; 
  }


  wait(20,msec);
}
}



//test wallstakes macros using IME's

void WallStakesMacro(double targetPosition){

  double kp = 0.1;

  WallStakes.resetPosition();
  WallStakes.setPosition(0,degrees);

  double realTarget = (targetPosition*15);
  double error = 0.0;
  double output = 0.0;


  double tolerance = 1.0;
  double speedLimit = 50;


  while (true) {
  // get the error
  double currentPosition = WallStakes.position(degrees); // motor positions
  error = realTarget - currentPosition;


  // output
  output = kp * error;


  if (output > 100.0) {
  output = 100.0;
  } else if (output < -100.0) {
  output = -100.0;
  }

  output *= speedLimit; 


  // spins motors
  WallStakes.spin(forward, output, percent);

  // checks to see if within tolerance
  if (fabs(error) < tolerance) {
  // Stop all motors

  wait(20,msec); // wait so commands have time to work
  break; // 
  }


  wait(20,msec); 
  }

}

//Old P function, unused now

void driveForward(double targetPosition,double speed,double kp){
  // reset the motor positions
  leftFront.resetPosition();
  leftMid.resetPosition();
  leftBack.resetPosition();
  rightFront.resetPosition();
  rightMid.resetPosition();
  rightBack.resetPosition();


  leftFront.stop(brake);
  leftMid.stop(brake);
  leftBack.stop(brake);
  rightFront.stop(brake);
  rightMid.stop(brake);
  rightBack.stop(brake);

  //double kp = 0.7;


  double realTarget = (targetPosition*15);
  double error = 0.0;
  double output = 0.0;


  double tolerance = 5.0;
  double speedLimit = (speed/200); //max speed cap is 50 percent


  while (true) {
  //error
  double currentPosition = (leftFront.position(degrees) + rightFront.position(degrees)) / 2; // dt motor positions
  error = realTarget - currentPosition;



  output = kp * error;


  //limits output to |100|
  if (output > 100.0) {
  output = 100.0;
  } else if (output < -100.0) {
  output = -100.0;
  }




  output *= speedLimit; // multiplies value by speed limit to get the right speed


  // spins motors
  leftFront.spin(forward, output, percent);
  leftMid.spin(forward, output, percent);
  leftBack.spin(forward, output, percent);
  rightFront.spin(forward, output, percent);
  rightMid.spin(forward, output, percent);
  rightBack.spin(forward, output, percent);
  // checks to see if within tolerance
  if (fabs(error) < tolerance) {
  // Stop all motors
  leftFront.stop();
  leftMid.stop();
  leftBack.stop(brake); //only back wheels stopped to prevent them from bouncing up
  rightFront.stop();
  rightMid.stop();
  rightBack.stop(brake);
  wait(20,msec); // wait so commands have time to work
  break; 
  }


  wait(20,msec); 

  }
}


vex::task antiJamCode() {

  while(1){

    wait(10,msec);

    Controller1.rumble("...");
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);

    Controller1.Screen.print(Upper.current());

    if (Upper.current()>2) {
    
     wait(500,msec);

    if (Upper.current()>2) {

      Controller1.Screen.print(Upper.current());
      Controller1.rumble("...");
      Upper.spin(reverse,100,percent);
      wait(250,msec);
      Upper.spin(forward,100,percent);

      }
    }
  }
 }



 double intakestopthreesec = 0.5; //variable to stop antijam from working in corners (current spike always triggers it)

vex::task ColorSortRed() {


  while(1){

    colorsorter1.setLightPower(100,percent);
    colorsorter2.setLightPower(100,percent);
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);

    //Controller1.Screen.print(Intake.current()); //debugging
    
    
    Controller1.Screen.print(colorsorter1.color());
    
    if (Intake.current()>2.5 and intakestopthreesec==0.5) {
    wait(500,msec); //to prevent antijams for momentary high voltage.
    if (Intake.current()>2.5) {
    
    Controller1.Screen.print(Intake.current());
    Intake.spin(reverse,100,percent);
    wait(250,msec);
    Intake.spin(forward,100,percent);
    wait(250,msec);
    
    }
    }
        
    if (colorsorter1.color() == red and colorsorter1.isNearObject() or colorsorter2.color() == red and colorsorter2.isNearObject()) {
    
    Intake.stop();
    sorter.set(true);
    wait(100,msec);

    Intake.spin(forward,100,percent);
    
    wait(1000,msec);
    
    sorter.set(false);
    
    
    Controller1.rumble("..."); //lets driver know that the ring is sorting
    
    }
    
    /*
    else if (intakestopthreesec > 0.5) {
    
    wait(3000,msec);
    intakestopthreesec = 0.5;
    }
    */
  }

}

vex::task ColorSortBlue() {

  while(1){

    colorsorter1.setLightPower(100,percent);
    colorsorter2.setLightPower(100,percent);
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    //Controller1.Screen.print(Intake.current());

    Controller1.Screen.print(colorsorter1.color());

    if (Intake.current()>2.4 and intakestopthreesec==0.5) {
    Controller1.rumble("...");
    wait(500,msec);
    if (Intake.current()>2.5) {

    Controller1.Screen.print(Intake.current());
    Intake.spin(reverse,100,percent);
    wait(250,msec);
    Intake.spin(forward,100,percent);
    wait(250,msec);

    }
    }

    if (colorsorter1.color() == blue and colorsorter1.isNearObject() or colorsorter2.color() == blue and colorsorter2.isNearObject()) {

    Intake.stop();
    sorter.set(true);
    wait(100,msec);
    Intake.spin(forward,100,percent);
    wait(1000,msec);

    sorter.set(false);

    Controller1.rumble("..."); //lets me know that ring is sorting

      }
   }

}





int driveFunction() {
int count = 0;

double drivespeed = 1;
  while(true) {

    //these combination of buttons turn the robot to a certain angle so auton setup is precise. (run before a match)
    if(Controller1.ButtonLeft.pressing() and Controller1.ButtonY.pressing()){

    Controller1.rumble("...");
    // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
    chassis.set_turn_constants(4, .3, .03, 3, 15);
    chassis.set_turn_exit_conditions(1, 1000, 3000);
    chassis.turn_to_angle(324);
    chassis.set_turn_exit_conditions(2, 100, 750);
    Controller1.rumble("...");

    }

    else if (Controller1.ButtonRight.pressing() and Controller1.ButtonY.pressing()){

    Controller1.rumble("...");
    chassis.set_turn_constants(4, .3, .03, 3, 15);
    chassis.set_turn_exit_conditions(1, 1000, 3000);
    chassis.turn_to_angle(36);
    chassis.set_turn_exit_conditions(2, 100, 750);
    Controller1.rumble("...");

    }

    else{

    //standard arcade drive
    float throttle = deadband(controller(primary).Axis3.value(), 12);
    float turn = deadband(controller(primary).Axis1.value(), 12);
    leftFront.spin(fwd, to_volt(throttle+turn), volt);
    leftMid.spin(fwd, to_volt(throttle+turn), volt);
    leftBack.spin(fwd, to_volt(throttle+turn), volt);
    rightFront.spin(fwd, to_volt(throttle-turn), volt);
    rightMid.spin(fwd, to_volt(throttle-turn), volt);
    rightBack.spin(fwd, to_volt(throttle-turn), volt);

    }

    this_thread::sleep_for(10);
  }

  return(0);
}




int intakeFunction() {
int count = 0;

  while(true) {
    Intake.setVelocity(100, percent);
    Upper.setVelocity(100,percent);

    if(Controller1.ButtonR1.pressing()){
      Intake.spin(forward);
      Upper.spin(forward);
    }
    else if(Controller1.ButtonR2.pressing()){
      Intake.spin(reverse);
      Upper.spin(reverse);
    }
    else{
      Intake.stop();
      Upper.stop();
    }

    this_thread::sleep_for(10);
  }

  return(0);
}


int ThrowAwayRedFunction() {

  int count = 0;

  sorter.set(false);

  while(true) {
    colorsorter1.setLightPower(100,percent);

    if (colorsorter1.color() == red) {

      sorter.set(true);
      wait(750,msec);
      sorter.set(false);

    }

    this_thread::sleep_for(10);
  }

  return(0);
}


int rumbleFunction() {

 int count = 0;


while(true) {

    color redring = color(229, 59, 59);
    color blueright = color(0, 182, 246);

    wait(10,msec);

    colorsorter1.setLightPower(100,percent);

    //lets driver know that intake is jammed

    if (Upper.current()>2.7) {

      Controller1.Screen.print(Upper.current());
      Controller1.rumble("...");
      Upper.spin(reverse,100,percent);
      wait(250,msec);
      Upper.spin(forward,100,percent);
      wait(250,msec);

    }

    else if (colorsorter1.color() == green) {

      Controller1.rumble("...");
      wait(50,msec);
    }
  }

  return(0);

}


int pullerFunction(){

  int count = 0;

  while(true){

    Puller.set(false);

    if(Controller1.ButtonB.pressing()){

      Tipper.set(true);

    Controller1.rumble("...");
    }

    else if(Controller1.ButtonY.pressing()){

      Tipper.set(false);
    }

    else if(Controller1.ButtonLeft.pressing()){

      Puller.set(false);

    }

    this_thread::sleep_for(10);
  }   

  return(0);
}




int WallStakesFunction(){


  int count = 1;


  while(true){

    this_thread::sleep_for(10); //gives time for other tasks to run

    if(Controller1.ButtonRight.pressing()){

        while(rotationSensor.angle(degrees) > 13) {
          WallStakes.spin(reverse, 100, percent);
        }
        while(rotationSensor.angle(degrees) < 13) {
          WallStakes.spin(forward, 100, percent);

        while(rotationSensor.angle(degrees) > 13) {
         WallStakes.spin(reverse, 100, percent);
        }
        }

        WallStakes.stop(hold);

        /*
        //Worst Case Macro
        WallStakes.setVelocity(100,percent);
        WallStakes.spinFor(110,degrees);
        */

        //WallStakes.setVelocity(10,percent);

    }
    else if(Controller1.ButtonL1.pressing()){

      WallStakes.setVelocity(100, percent);
      WallStakes.spin(forward);

    }
    else if(Controller1.ButtonL2.pressing()){

      WallStakes.setVelocity(100, percent);
      WallStakes.spin(reverse);

    }
    else{

    WallStakes.stop(hold);

    }
  }
}


int doinkerFunction(){

  int count = 1;

  while(true){

    this_thread::sleep_for(10);
    if(Controller1.ButtonUp.pressing()){

      //DoinkerRight.set(true); //change before every match depending on the alliance color
      DoinkerLeft.set(true);
      wait(10,msec);

    }
    else if(Controller1.ButtonDown.pressing()){

      DoinkerRight.set(false);
      DoinkerLeft.set(false);
      wait(10,msec);
    }
  }

return(0);

}

int tempFunction(){

int count = 1;

  while(true){

    this_thread::sleep_for(10);

    if (rightMid.temperature(temperatureUnits::fahrenheit) > 150){
      Controller1.rumble("...");
      Controller1.Screen.print("STOP DRIVING!!!");
      }
    else if(rightMid.temperature(temperatureUnits::fahrenheit) < 150){

    }
    else{

    }
  }

return(0);

}

void TempCode(){

  if (rightMid.temperature(temperatureUnits::fahrenheit) > 130){
    Controller1.rumble("...");
    Controller1.Screen.print("STOP DRIVING!!!");
  }
    else if(rightMid.temperature(temperatureUnits::fahrenheit) < 120){
  }
  else{

  }
}

int mogoFunction(){


int count = 1;


while(true){

if(Controller1.ButtonX.pressing()){

MOGO.set(true);
mogo.set(true);

}

else if(Controller1.ButtonA.pressing()){

MOGO.set(false);
mogo.set(false);

}

}


return(0);

}









int selectorFunction(){


int count = 1;
while(true){

if(Controller1.ButtonRight.pressing() and Controller1.ButtonY.pressing()){

chassis.turn_to_angle(30);

}


if(Controller1.ButtonUp.pressing() and Controller1.ButtonY.pressing()){


chassis.turn_to_angle(340);
}


if(Controller1.ButtonDown.pressing()){
Controller1.rumble("...");




}


}


return(0);


}








//auton selector
int current_auton_selection = 0;
bool auto_started = false;


void pre_auton(void) {
vexcodeInit();


Inertial.calibrate();

while (Inertial.isCalibrating() == true){
Controller1.rumble("...");
wait(500,msec);
}

current_auton_selection = 0;


default_constants();

while(auto_started == false){
//Brain.Screen.clearScreen();
switch(current_auton_selection){
case 0:

  //Brain.Screen.printAt(50, 50, "SOLO AWP RED");

  SoloAwpRedPicture(); //puts a custom picure on the brain screen from pictures.cpp

break;

case 1:

  Brain.Screen.printAt(50, 50, "Red Pos 6 Ring");
  wait(1000,msec);

case 2:

  Brain.Screen.printAt(50, 50, "Red Pos 1+2+1 AWP");

break;

case 3:

  Brain.Screen.printAt(50, 50, "Red Left 5 Ring Quals (Bar Touch)");

break;


case 4:

  Brain.Screen.printAt(50, 50, "Red Left Ally + 5 (Bar Touch)");

break;

case 5:

  Brain.Screen.printAt(50, 50, "Red Left Ally + 6 Elims");

break;

case 6:

  Brain.Screen.printAt(50, 50, "Solo Awp Blue (Bar Touch)");

break;

case 7:

  Brain.Screen.printAt(50, 50, "Blue Left 6 Ring POS");

break;

case 8:

  Brain.Screen.printAt(50, 50, "Blue Right 1+2+1 Solo Wp");

break;

case 9:

  Brain.Screen.printAt(50, 50, "Blue Right 5 Ring Quals (Bar Touch)");

break;

case 10:

  Brain.Screen.printAt(50, 50, "Blue Right Ally + 3 (Bar Touch)");

break;

case 11:

  Brain.Screen.printAt(50, 50, "Blue Right 4+1 Elims");

break;

case 12:

  Brain.Screen.printAt(50, 50, "Skills");

break;

case 13:

Brain.Screen.printAt(50, 50, "Testing");

break;


}

if(Brain.Screen.pressing()){

  while(Brain.Screen.pressing()) {}

  Brain.Screen.clearScreen();
  current_auton_selection ++;
} 

else if (current_auton_selection == 14){
  current_auton_selection = 0;
}
task::sleep(10);
  }

}








void autonomous(void) {
auto_started = true;
switch(current_auton_selection){
case 13: //SOLO AWP RED - 1+3+2
WallStakes.spin(forward,75,percent);

default_constants();

chassis.drive_distance(2);

default_constants();

//Motion Chaining Tuning

default_constants();
// Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).

WallStakes.stop();

chassis.drive_distance(-16,20);

WallStakes.spin(reverse,100,percent);

mogo.set(true);
MOGO.set(true);

wait(250,msec);

chassis.turn_to_angle(170); 

WallStakes.stop(coast);




Intake.spin(forward,100,percent);

default_constants();
chassis.drive_distance(8.2);



chassis.turn_to_angle(135);


chassis.drive_distance(5.5,130);

wait(250,msec);



chassis.drive_distance(-13,175);

chassis.drive_distance(8,135);

chassis.drive_distance(-7);

chassis.turn_to_angle(300);

break;


case 1: //Red Right 6 Ring



chassis.drive_distance(-9);


chassis.drive_distance(-4);


mogo.set(true);
MOGO.set(true);


wait(250,msec);


//Intake.spin(forward,100,percent);


//wait(300,msec);


chassis.turn_to_angle(133);




Intake.spinFor(100,degrees,false);




chassis.drive_distance(8.2);


DoinkerRight.set(true);


wait(250,msec);


chassis.turn_to_angle(150);


chassis.drive_distance(3);




DoinkerLeft.set(true);


wait(100,msec);


chassis.set_drive_exit_conditions(1, 75, 5000);


chassis.drive_distance(-18,160);


DoinkerLeft.set(false);
DoinkerRight.set(false);
  

wait(250,msec);


Intake.spin(forward,100,percent);


chassis.drive_distance(8,140);

Intake.stop();

chassis.turn_to_angle(270);

Intake.spin(forward,100,percent);

chassis.drive_max_voltage = 4;


chassis.drive_distance(16,250);




default_constants();



chassis.drive_distance(-6.5,225);


chassis.turn_to_angle(313);




chassis.drive_distance(11);

Intake.spin(forward,50,percent);


wait(500,msec);

Intake.spin(forward,100,percent);




codestopper129 = true;


chassis.set_drive_exit_conditions(1, 75, 1000); //the last value has to be tuned so it leaves right after intaking the ring
wait(250,msec);

chassis.drive_max_voltage=6;


intakestopthreesec=1;

Intake.spin(forward,100,percent);

chassis.drive_distance(9);

wait(150,msec);
//chassis.drive_distance(-2);

//chassis.drive_distance(3);



default_constants();





chassis.drive_distance(-2);


chassis.set_drive_exit_conditions(1, 75, 500); //the last value has to be tuned so it leaves right after intaking the ring

chassis.drive_distance(3);

/*

chassis.drive_distance(-10,305);

DoinkerRight.set(true);

chassis.drive_distance(13,310);
*/

chassis.drive_distance(-3);

Puller.set(true);


wait(250,msec);


chassis.set_drive_exit_conditions(1, 75, 750); //the last value has to be tuned so it leaves right after intaking the ring


chassis.drive_distance(5);



wait(250,msec);


Puller.set(false);



chassis.set_drive_exit_conditions(1, 75, 5000);
chassis.drive_distance(-9,330);




DoinkerRight.set(true);
wait(250,msec);


chassis.set_drive_exit_conditions(1, 75, 300);
chassis.drive_distance(3);


chassis.turn_to_angle(150);


wait(100,msec);


mogo.set(false);
MOGO.set(false);




chassis.drive_distance(-3);

/*
*/

break;

case 0: //Red Pos 1+2+1 SOLO WP

intakestopthreesec = 0.5;


chassis.set_drive_exit_conditions(1, 75, 5000);

Tipper.set(true);

WallStakes.spinFor(500,degrees,false);

chassis.drive_distance(18,23);

WallStakes.spin(forward,100,percent);

chassis.drive_distance(-10,20);

Tipper.set(false);

wait(200,msec);

WallStakes.spin(reverse,100,percent);
wait(400,msec);

chassis.drive_distance(-3);

chassis.turn_to_angle(205);

chassis.drive_max_voltage=4;

chassis.drive_distance(-10);

WallStakes.stop(hold);

mogo.set(true);
MOGO.set(true);

wait(250,msec);

Intake.spin(forward,100,percent);

wait(300,msec);

mogo.set(false);

MOGO.set(false);

chassis.turn_to_angle(71);

chassis.drive_distance(-10);

mogo.set(true);
MOGO.set(true);

wait(250,msec);

default_constants();

chassis.drive_distance(15,75);

wait(250,msec);

chassis.drive_distance(-10,50);

chassis.turn_to_angle(135);



chassis.set_drive_exit_conditions(1, 75, 1750); //the last value has to be tuned so it leaves right after intaking the ring
wait(250,msec);

chassis.drive_max_voltage=6;


intakestopthreesec=1;

Intake.spin(forward,100,percent);

chassis.drive_distance(13);

chassis.drive_max_voltage=9;

chassis.drive_distance(5);

wait(500,msec);


default_constants();




chassis.drive_distance(-2.5);


Puller.set(true);


wait(250,msec);


chassis.set_drive_exit_conditions(1, 75, 750); //the last value has to be tuned so it leaves right after intaking the ring


chassis.drive_distance(5);


wait(250,msec);


Puller.set(false);


chassis.set_drive_exit_conditions(1, 75, 5000);
chassis.drive_distance(-5);

/*

WallStakes.spin(forward,80,percent);

default_constants();

chassis.drive_distance(3);

wait(250,msec);

WallStakes.spin(reverse,100,percent);

//Motion Chaining Tuning

default_constants();
// Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
chassis.set_drive_constants(12, 8, 0, 0.0, 0);
chassis.drive_distance(-10);

WallStakes.stop();
default_constants();
chassis.drive_distance(-3);

mogo.set(true);
MOGO.set(true);

chassis.turn_to_angle(245);

chassis.drive_distance(10);

DoinkerRight.set(true);

wait(250,msec);

//Drive Forward Default

default_constants();
chassis.drive_distance(-10,170);

DoinkerRight.set(false);

chassis.left_swing_to_angle(270);

default_constants();
chassis.drive_distance(13);

wait(500,msec);

default_constants();
chassis.drive_distance(-10,210);

chassis.turn_to_angle(315);

chassis.drive_distance(10);

wait(250,msec);

chassis.drive_distance(-3);

Puller.set(true);

chassis.drive_distance(4);

*/



break;


case 3: //Red Left 5 Ring Quals (Bar Touch)



break;





case 4: //Red Left Ally + 4 (Bar Touch)



break;




case 5: //Red Left 1 + 6 Elims


WallStakes.spin(forward,75,percent);

default_constants();

chassis.drive_distance(2);

wait(100,msec);

default_constants();


WallStakes.stop();

//chassis.drive_distance(-12,17);

chassis.drive_distance(-12,12);

chassis.drive_distance(-4);

WallStakes.spin(reverse,100,percent);

mogo.set(true);
MOGO.set(true);




wait(200,msec);

chassis.turn_to_angle(169);

WallStakes.stop(coast);

Intake.spin(forward,100,percent);

default_constants();

//chassis.drive_distance(9);

chassis.drive_distance(8.4);

chassis.turn_to_angle(135);

chassis.drive_distance(5.5,130);

//chassis.drive_distance(7,130);

//wait(250,msec);

//chassis.drive_distance(-13,175);
chassis.drive_distance(-13,165);

chassis.drive_distance(8,135);


chassis.drive_distance(-7.5);



//chassis.turn_to_angle(78);

chassis.turn_to_angle(82);

chassis.drive_distance(17);

codestopper129 = false;

intakestopthreesec = 1;

chassis.set_drive_exit_conditions(1, 75, 750); //the last value has to be tuned so it leaves right after intaking the ring

chassis.drive_distance(6);


chassis.drive_distance(-2);

chassis.drive_distance(3);

chassis.drive_distance(-3);


Puller.set(true);

wait(100,msec);

chassis.set_drive_exit_conditions(1, 75, 750); //the last value has to be tuned so it leaves right after intaking the ring

chassis.drive_distance(5);



chassis.set_drive_exit_conditions(1, 30, 5000);

Puller.set(false);

wait(100,msec);

chassis.drive_distance(-35,120);

/*

chassis.drive_distance(-18,95);


//Quals Bar Touch


  chassis.set_turn_exit_conditions(1, 30, 600);


chassis.turn_to_angle(240);

WallStakes.spin(forward,60,percent);

chassis.drive_max_voltage=5;

chassis.drive_distance(20,240);
*/


/*
chassis.drive_distance(-18,120);

chassis.turn_to_angle(295);


intakestopthreesec = 1;

//Elims 6th Ring
Puller.set(true);

chassis.set_drive_exit_conditions(1, 75, 2000); //the last value has to be tuned so it leaves right after intaking the ring

chassis.drive_max_voltage = 3;

chassis.drive_distance(6);

Puller.set(false);

chassis.drive_distance(-3);





/*

chassis.drive_distance(-3);

Puller.set(true);
wait(250,msec);


chassis.drive_distance(4);

Puller.set(false);

wait(250,msec);

chassis.drive_distance(-10,100);

Puller.set(true);

chassis.turn_to_angle(290);

chassis.drive_distance(10,280);
Puller.set(false);
wait(250,msec);

chassis.drive_distance(-3);
*/
break;

case 6: // SIG SOLO AWP Blue

break;


case 7: //Blue Left 6 Ring Pos


chassis.drive_distance(-9);

chassis.drive_distance(-4);

mogo.set(true);
MOGO.set(true);

wait(250,msec);

//Intake.spin(forward,100,percent);

//wait(300,msec);

chassis.turn_to_angle(226);


Intake.spinFor(100,degrees,false);


chassis.drive_distance(8.6);

DoinkerLeft.set(true);

wait(250,msec);

chassis.turn_to_angle(207);

chassis.drive_distance(2.3);


DoinkerRight.set(true);

wait(100,msec);

chassis.set_drive_exit_conditions(1, 75, 5000);

chassis.drive_distance(-18,200);

DoinkerLeft.set(false);
DoinkerRight.set(false);

wait(250,msec);

Intake.spin(forward,100,percent);

chassis.drive_distance(8,220);

Intake.stop();

chassis.turn_to_angle(90);


Intake.spin(forward,100,percent);

chassis.drive_distance(16,110);




chassis.drive_distance(-7.5,135);

chassis.turn_to_angle(47);


chassis.drive_distance(11);
Intake.spin(forward,50,percent);

wait(500,msec);


Intake.spin(forward,100,percent);
codestopper129 = true;

chassis.set_drive_exit_conditions(1, 75, 1500); //the last value has to be tuned so it leaves right after intaking the ring

chassis.drive_max_voltage=6;

intakestopthreesec=1;

Intake.spin(forward,100,percent);

chassis.drive_distance(9);

wait(250,msec);
//chassis.drive_distance(-2);

//chassis.drive_distance(3);


chassis.drive_distance(-15);


/*

//Elims

wait(250,msec);

default_constants();

chassis.drive_distance(-2.5);

Puller.set(true);

wait(250,msec);

chassis.set_drive_exit_conditions(1, 75, 750); //the last value has to be tuned so it leaves right after intaking the ring

chassis.drive_distance(5);

wait(250,msec);

Puller.set(false);

chassis.set_drive_exit_conditions(1, 75, 5000);


chassis.drive_distance(-7,30);
*/


/*


DoinkerLeft.set(true);
wait(250,msec);

chassis.set_drive_exit_conditions(1, 75, 300);
chassis.drive_distance(3);

chassis.turn_to_angle(225);

Intake.spin(reverse,20,percent);

wait(100,msec);

mogo.set(false);
MOGO.set(false);


/*
*/


//chassis.drive_distance(13);


break;


case 8: //Blue Left 1+2+1 SOLO WP




break;


case 9: //Blue Right 5 Ring Quals (Bar Touch)

break;


case 10: //Blue Right Ally + 4 (Bar Touch)





break;


case 11: //Blue Right 6+1 Elims


WallStakes.spin(forward,75,percent);


default_constants();


chassis.drive_distance(2);


wait(100,msec);


default_constants();




WallStakes.stop();


chassis.drive_distance(-12,348);


chassis.drive_distance(-4);


WallStakes.spin(reverse,100,percent);


mogo.set(true);
MOGO.set(true);








wait(200,msec);


chassis.turn_to_angle(194);


WallStakes.stop(coast);


Intake.spin(forward,100,percent);


default_constants();


//chassis.drive_distance(9);


chassis.drive_distance(8.2);


chassis.turn_to_angle(225);

chassis.drive_distance(5.5,230);


//chassis.drive_distance(7,230);


//wait(250,msec);


chassis.drive_distance(-13,185);


chassis.drive_distance(8,225);




chassis.drive_distance(-6);






//chassis.turn_to_angle(78);


chassis.turn_to_angle(283);


chassis.drive_distance(17);


codestopper129 = false;


intakestopthreesec = 1;


//chassis.set_drive_exit_conditions(1, 75, 1250); //the last value has to be tuned so it leaves right after intaking the ring
chassis.set_drive_exit_conditions(1, 75, 1000); //the last value has to be tuned so it leaves right after intaking the ring

chassis.drive_distance(6);



chassis.drive_distance(-2);

chassis.drive_distance(3);


/*

chassis.drive_distance(-2.5);




Puller.set(true);


wait(100,msec);


chassis.set_drive_exit_conditions(1, 75, 750); //the last value has to be tuned so it leaves right after intaking the ring


chassis.drive_distance(5);


Puller.set(false);


wait(100,msec);

chassis.set_drive_exit_conditions(1, 30, 5000);
*/

chassis.drive_distance(-20,260);

//chassis.turn_to_angle(100);

//chassis.drive_distance(10);



//Quals Bar Touch

chassis.set_turn_exit_conditions(1, 30, 600);

chassis.turn_to_angle(120);

WallStakes.spin(forward,70,percent);

chassis.drive_max_voltage=6;

chassis.drive_distance(20);



/*

//Elims Auton 

chassis.turn_to_angle(68);




Puller.set(true);


chassis.set_drive_exit_conditions(1, 75, 2000); //the last value has to be tuned so it leaves right after intaking the ring


chassis.drive_max_voltage = 3;


chassis.drive_distance(6);


Puller.set(false);


chassis.drive_distance(-3);





/*
*/
break;


case 12: // Skills (not for worlds)

Intake.spin(forward,100,percent);
wait(500,msec);

default_constants();
chassis.drive_distance(5,0);

chassis.turn_to_angle(270);

default_constants();
// Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
chassis.set_drive_constants(12, 8, 0, 0.0, 0);
chassis.drive_distance(-10,270);
default_constants();
chassis.drive_distance(-3,270);



break;

case 2: // Testing

default_constants();

chassis.turn_to_angle(90);

/*

chassis.set_coordinates(0, 0, 0);

chassis.drive_to_point(10,15);

*/

break;
}
}





/*---------------------------------------------------------------------------*/
/* */
/* User Control Task */
/* */
/* This task is used to control your robot during the user control phase of */
/* a VEX Competition. */
/* */
/* You must modify the code to add your own robot specific commands here. */
/*---------------------------------------------------------------------------*/




void usercontrol(void) {
// User control code here, inside the loop




vex::task driveTask = vex::task( driveFunction );


vex::task intakeTask = vex::task( intakeFunction );


vex::task pullerTask = vex::task( pullerFunction );


//vex::task ThrowAwayRedTask = vex::task( ThrowAwayRedFunction );


//vex::task rumbleTask = vex::task( rumbleFunction ); //this function is just for testing tasks


//vex::task ThrowAwayBlueTask = vex::task( ThrowAwayBlueFunction );


vex::task WallStakesTask = vex::task( WallStakesFunction );


vex::task doinkerTask = vex::task( doinkerFunction );


vex::task mogoTask = vex::task( mogoFunction );


//vex::task selectorTask = vex::task( selectorFunction );

while (1) {



//all switched to tasks

//intakeCode();
//positionCode();
//WallStakesCode();
//selectorCode();
//descoreCode();
//ThrowAwayRed();
//ThrowAwayBlue();
//DoinkerCode();
// MogoCode();
//TempCode();
//arcadeDrive();
//chassis.control_arcade();

wait(20, msec); // Sleep the task for a short amount of time to
// prevent wasted resources.
}
}

 
//
// Main will set up the competition functions and callbacks.
//


int main() {

  // Set up callbacks for autonomous and driver control periods.
  colorsorter1.integrationTime(5);
  colorsorter2.integrationTime(5);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();


  //ColorSortBlue and ColorSortRed functions need to be switched before every match

  //ColorSortBlue();

  ColorSortRed();

  //antiJamCode(); //now included in colorsortfunctions

  // Run the pre-autonomous function.

  //ColorSortRed();
  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}




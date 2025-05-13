#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor rightFront = motor(PORT11,ratio18_1, false);
motor rightBack = motor(PORT1, ratio18_1, false);
motor rightMid = motor(PORT14, ratio18_1, true);
motor leftFront = motor(PORT12 , ratio18_1, true);
motor leftBack = motor(PORT13, ratio18_1, true);
motor leftMid = motor(PORT3, ratio18_1, false);
controller Controller1 = controller(primary);
motor Intake = motor(PORT16, ratio36_1, true); 
inertial Inertial = inertial(PORT20); 
digital_out MOGO = digital_out(Brain.ThreeWirePort.A);
distance colordistance = distance(PORT2);
motor Upper = motor(PORT2, ratio18_1, true);
digital_out mogo = digital_out(Brain.ThreeWirePort.B);
optical colorsorter1 = optical(PORT19);  
optical colorsorter2 = optical(PORT10);
motor WallStakes = motor(PORT15, ratio18_1, true);
rotation rotationSensor = rotation(PORT3,false);
digital_out Puller = digital_out(Brain.ThreeWirePort.C);
digital_out DoinkerRight = digital_out(Brain.ThreeWirePort.H);
digital_out DoinkerLeft = digital_out(Brain.ThreeWirePort.E);
digital_out Doinker = digital_out(Brain.ThreeWirePort.G); //change later
digital_out sorter = digital_out(Brain.ThreeWirePort.F);
digital_out Tipper = digital_out(Brain.ThreeWirePort.D);




// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}

#include <tr1/math.h>
#include "main.h"
#include "config.hpp"

//method for driveTrain
void driveTrain (float unit){
  lfront.move(unit);
  lback.move(unit);
  rfront.move(unit);
  rback.move(unit);
}

//method for brake
void baseHold (void){
  lfront.set_brake_mode(MOTOR_BRAKE_HOLD);
  lback.set_brake_mode(MOTOR_BRAKE_HOLD);
  rfront.set_brake_mode(MOTOR_BRAKE_HOLD);
  rback.set_brake_mode(MOTOR_BRAKE_HOLD);
}

//total power sent to motors for PID loop
float power;
//variable to alter power to adjust side that is off (if applicable)
float powerCorrection;

//slippage methods (dual encoder)
/*
//must be in forward direction (adjust power if right side is ahead)
void correctRight(power, powerCorrection){
  lfront.move(power);
  lback.move(power);
  rfront.move(powerCorrection);
  rback.move(powerCorrection);
 }

 //adjust power if left side is ahead
 void correctLeft (power, powerCorrection){
   lfront.move(powerCorrection);
   lback.move(powerCorrection);
   rfront.move(power);
   rback.move(power);
 }
*/

//state forward or backward when calling function in robotMove direction parameter
int forward = 1;
int backward = -1;

//target in inches
void robotMove (int direction, double target){

//PID variables
float Kp;
float proportion;

float Ki;
float integral;

float Kd;
float derivative;
float prevError;

float error;


//diameter of wheel in inches
double wheelDiameter = 4.125;
//Pi*diameter or 2*Pi*radius
double circumference = wheelDiameter * M_PI;
//input converted to ticks
//inches to ticks
target = (360*target) / circumference;

//constants
Kp = .5;
Ki = .001;
Kd = .03;


//resets encoder rotation to 0 so that if method was previously called, the
//encoder starts from 0 (absolute postion, not relative)
encoder.reset();
//lEncoder.reset();
//rEncoder.reset();

//slippage variables
/*
//constant value to adjust slippage proportionally
float Ks;

//small value to scale down correction to a useful value
Ks = .01;

//bool determining if slippage is not negligible
bool slippageRight1 = false;
bool slippageLeft1 = false;

//determines error in slippage on one side of drive
double slippageRight;
double slippageLeft;
*/

/*
//use if quad encoders not availible
//reset encoders
rFront.resetRotation();
rBack.resetRotation();
lFront.resetRotation();
lBack.resetRotation();
*/

    //PID loop with straight line correction
    while(error > target){

      //average error (using 2 encoders)
      /*
        //error is difference between target and current position
        //takes average between 2 encoders
        error = target - ((lEncoder.get_value() + rEncoder.get_value())/2);
*/
      error = target - encoder.get_value();
      //set proportion = updated error value (makes code easier to read)
      proportion = error;


        //set a limit to integral term to prevent wind up (stop calculations once target is over shot or achieved)
        if((error = 0) || error < target){
            integral = 0;
        }
        //since integral is the sum of errors, prevent it from getting too big (allow it to start calculating at a certain point)
        else if (error > 3000){
            integral = 0;
        }
        else{
          //I term is the sum of all errors (starts with just error then keeps adding)
          integral += error;
        }


        // derivative term is rate of change of error (change in error over time) -- since
        // time of loop is constant, time is not taken into account
        //slope of error v time graph negative, therefore derivative is negative
        derivative = error - prevError;
        //new prevError is the error of the current loop after derivative is calculated (value gets saved each cycle)
        prevError = error;

        //power value is error times constant times 1 or -1, direction based on parameter
        power = (proportion*Kp + integral*Ki + derivative*Kd)*direction;

        //sets max power, since PID output translates to rpm, max power is
        //200 rpm
        if(power > 127){
            power = 127;
        } else if (power < -127){
            power = -127;
        }

//slippage correction with dual encoder set up
/*
     slippageRight = lEncoder.get_value() - rEncoder.get_value();
     slippageLeft = rEncoder.get_value() - lEncoder.get_value();

     //if left side or right side is not lagging, else determines which side is lagging and sets it to true
    if((slippageLeft < 200) || (slippageRight < 200)){
        slippageLeft1 = false;
        slippageRight1= false;
    }
    else if(slippageLeft > 300) {
        slippageLeft1 = true;
    }
    else if (slippageRight > 300){
        slippageRight1 = true;
    }

  //check to see if there is slippage, if false then there is slippage
  if(slippageRight == true){

      //while slippage error is not negligible
      while(slippageRight != 0){
           //current power - how much power is needed on lagging side to level it out
            powerCorrection = power - (slippageRight*Ks);
            //increase power of the side that is lagging (current power - added slippage correction)
            correctRight(power, powerCorrection);
            pros::Task::delay(20);
           }
    }

     if((slippageLeft = true)){
           //while slippage error is not negligible
           while(slippageLeft != 0){
             // determines how much power is needed on lagging side to level it out
              powerCorrection = power - (slippageLeft*Ks) ;
            //decrease power of the side that is ahead (current power - powerCorrection)
            correctLeft(power, powerCorrection);
            pros::Task::delay(20);
           }
    }
*/

        //Motor power output based on PID loop feed back
        driveTrain(power);

//brain debugging
        /*
        //allow for debugging on screen
        Brain.Screen.printAt( 10, 40, “counts %.2f”, Encoder.rotation(rotationUnits::raw));
        Brain.Screen.printAt( 10, 60, “error %d”, error );
        Brain.Screen.printAt( 10, 100, “Power %d”, power );
        */

        //delay loop every 30 milliseconds
        pros::Task::delay(30);
    }

    //once pid loop ends, hold base in place
    baseHold();
}

//------------------------------------------------------------------------------------------------------------
//P-loop
void drive (double unit){
  //sets motors velocity from 127, -127 in P loop
  lfront.move(unit);
  lback.move(unit);
  rfront.move(unit);
  rback.move(unit);
}

//target is an integer in inches
void pLoop (int direction, double target){

    //p-loop variables (Kr variable name bc Kp already used)
    float Kr;
    float proportion;

    float error;

    //CONVERSION
    //diameter of wheel in inches
    double wheelDiameter = 4.125;
    //Pi*diameter or 2*Pi*radius
    double circumference = wheelDiameter * M_PI;
    //input converted to ticks
    //inches to ticks
    target = (360*target) / circumference;

    //TUNE constant value
    Kr = .5;


  while (error > target || error != 0){

      //update error each cycle (average between 2 sides)
      error = target - encoder.get_value();

      //make proportion = error (makes easier to read)
      proportion = error;
      //adjust proportion
      power = proportion*Kr*direction;

      drive(power);

      pros::Task::delay(20);

    }

  baseHold();

}

#include "main.h"
#include "config.hpp"



void autonomous() {

lfront.move(127);

pros::Task::delay(10000);
/*//direction (1 or -1) and target distance in inches
robotMove(1, 24);
*/

//p-loop
pLoop (1, 24);

pLoop (-1,24);


}

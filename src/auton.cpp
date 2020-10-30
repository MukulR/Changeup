#include "motordefs.hpp"
#include "Auton.hpp"

Auton::Auton(MotorDefs *md, bool ra){
    mtrDefs = md;
    redAlliance = ra;
    //ac = new AutonCommons(mtrDefs, imu);
}

Auton::~Auton() {
    // delete ac;
}


void Auton::runAuton(){

}

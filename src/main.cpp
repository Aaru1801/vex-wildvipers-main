#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/motors.hpp"

ASSET(plsworkman1_txt);

// ===========================
//  MOTOR SETUP
// ===========================
// LEFT DRIVE = ports 1,2,3  (ALL BLUE)
pros::MotorGroup left_motors({1, 2, 3}, pros::MotorGearset::blue);

// RIGHT DRIVE = ports 4,5,6 (ALL BLUE)
pros::MotorGroup right_motors({-4, -5, -6}, pros::MotorGearset::blue);
// NEGATIVE because right side must be reversed

// INTAKE = port 7
pros::Motor intake(7, pros::MotorGearset::blue);

// OUTTAKE = port 8
pros::Motor outtake(8, pros::MotorGearset::blue);

// ===========================
//  IMU
// ===========================
pros::Imu imu(10);

// ===========================
//  DRIVETRAIN (LEMLIB)
// ===========================
lemlib::Drivetrain drivetrain(
    &left_motors,
    &right_motors,
    10,                             // track width (inches)
    lemlib::Omniwheel::NEW_4,       // 4" omni wheels
    360,                            // drivetrain RPM (blue cart @ 4” wheels)
    2                               // horizontal drift
);

// LATERAL PID
lemlib::ControllerSettings lateral_controller(
    10, 0, 3,
    0,
    1, 100,
    3, 250,
    0
);

// ANGULAR PID
lemlib::ControllerSettings angular_controller(
    4, 0, 10,
    0,
    1, 100,
    2, 250,
    0
);

lemlib::OdomSensors sensors(
    nullptr,   // vertical tracking wheel
    nullptr,   // vertical tracking wheel 2
    nullptr,   // horizontal tracking wheel
    nullptr,   // horizontal tracking wheel 2
    &imu       // IMU pointer
);

// CHASSIS
lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    sensors
);

// CONTROLLER
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ===========================
//  INITIALIZE
// ===========================
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Initializing...");
    chassis.calibrate();      // calibrate IMU+odom
    pros::lcd::set_text(1, "Calibrated!");
}

void disabled() {}
void competition_initialize() {}

// ===========================
//  AUTONOMOUS
// ===========================
void autonomous() {
    chassis.setPose(-67.244, -22.929, 90);

    // Move forward to score
    chassis.moveToPose(-27.866, -22.389, 90, 2000);
    intake.move(127);

    // Move & angle
    chassis.moveToPose(-10.109, -10.109, 51.667, 1000);
}

// ===========================
//  OPERATOR CONTROL
// ===========================
void opcontrol() {

    // PNEUMATICS
    pros::ADIDigitalOut pistonA('A');   // main pneumatic: UP/DOWN
    pros::ADIDigitalOut pistonB('B');   // toggle pneumatic on button A

    bool pistB = false;                 // toggle state for piston B
    bool lastA = false;                 // prevents rapid toggle spam

    while (true) {

        // ====================
        // DRIVING: curvature control
        // ====================
        int leftY  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int throttle = -leftY;      // up = forward
        int turn     = -rightX;     // right = turn right

        chassis.curvature(throttle, turn);

        // ====================
        // INTAKE (PORT 7) - L1/L2
        // ====================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(-127);
        }
        else {
            intake.move(0);
        }

        // ====================
        // OUTTAKE (PORT 8) - R1/R2
        // ====================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            outtake.move(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            outtake.move(-127);
        }
        else {
            outtake.move(0);
        }

        // ====================
        // PNEUMATIC A (port A)
        // UP = extend
        // DOWN = retract
        // ====================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            pistonA.set_value(true);   // extend
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            pistonA.set_value(false);  // retract
        }

        // ====================
        // PNEUMATIC B (port B)
        // BUTTON A = toggle
        // ====================
        bool aNow = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        if (aNow && !lastA) {
            pistB = !pistB;
            pistonB.set_value(pistB);
        }
        lastA = aNow;

        // ====================
        // RUN AUTON IN DRIVER
        // ====================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            autonomous();
        }

        pros::delay(20);
    }
}
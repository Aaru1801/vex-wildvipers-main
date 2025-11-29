#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/motors.hpp"
#include <cmath>

// ===========================
//  MOTOR SETUP
// ===========================

// LEFT DRIVE = ports 1, 2, 3 (ALL BLUE)
pros::MotorGroup left_motors({1, 2, 3}, pros::MotorGearset::blue);

// RIGHT DRIVE = ports 4,5,6 (ALL BLUE)
pros::MotorGroup right_motors({-4, -5, -6}, pros::MotorGearset::blue);
// NEGATIVE because right side must be reversed

// INTAKE = port 7
pros::Motor intake(7, pros::MotorGearset::blue);

// OUTTAKE = port 8
pros::Motor outtake(8, pros::MotorGearset::blue);

// ===========================
//  IMU (must come BEFORE OdomSensors)
// ===========================
pros::Imu imu(10);

// ===========================
//  DRIVETRAIN SETUP (LEMLIB)
// ===========================
lemlib::Drivetrain drivetrain(
    &left_motors,
    &right_motors,
    10,                        // track width
    lemlib::Omniwheel::NEW_4,  // 4" omni wheels
    600,                       // RPM
    0                          // horizontal drift
);

lemlib::ControllerSettings lateral_controller(
    10, 0, 3,
    0,
    1, 100,
    3, 250,
    0
);

lemlib::ControllerSettings angular_controller(
    4, 0, 10,
    0,
    1, 100,
    2, 250,
    0
);

lemlib::OdomSensors sensors(
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    &imu    // now valid: imu is declared above
);

lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    sensors
);

// ===========================
//  IMU TURN HELPER
// ===========================
void turnDegrees(double targetDeg, int speed = 80) {
    // Zero the IMU rotation so current heading = 0
    imu.tare();
    pros::delay(50); // small delay to apply tare

    const double kTolerance = 2.0;    // degrees
    const int    kDelayMs   = 10;     // loop delay

    while (true) {
        double current = imu.get_rotation();        // current angle in degrees
        double error   = targetDeg - current;

        if (std::fabs(error) <= kTolerance) {
            break; // close enough
        }

        int dir = (error > 0) ? 1 : -1; // which way to turn

        left_motors.move(dir * speed);
        right_motors.move(-dir * speed);

        pros::delay(kDelayMs);
    }

    // stop motors at the end
    left_motors.move(0);
    right_motors.move(0);
}

// ===========================
// CONTROLLER
// ===========================
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// ===========================
//  INITIALIZE
// ===========================
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(0, "Calibrating IMU...");

    imu.reset();                      // start IMU calibration
    while (imu.is_calibrating()) {    // wait until done
        pros::delay(20);
    }

    pros::lcd::set_text(0, "Robot Ready");
}

void disabled() {}
void competition_initialize() {}

// ===========================
//  AUTONOMOUS
// ===========================
void autonomous() {
    // Example: drive forward 1 second, turn right 90°, then stop

    left_motors.move(127);
    right_motors.move(127);
    pros::delay(1000);   // 1000 ms = 1 second
    left_motors.move(0);
    right_motors.move(0);

    // Turn RIGHT 90 degrees using IMU
    left_motors.move(127);
    right_motors.move(-127);
    pros::delay(500);   // tune this value!
    left_motors.move(0);
    right_motors.move(0);

    left_motors.move(127);
    right_motors.move(127);
    pros::delay(1000);   // 1000 ms = 1 second
    left_motors.move(0);
    right_motors.move(0);

    left_motors.move(127);
    right_motors.move(-127);
    pros::delay(500);   // tune this value!
    left_motors.move(0);
    right_motors.move(0);

    left_motors.move(127);
    right_motors.move(127);
    pros::delay(500);   // 1000 ms = 1 second
    left_motors.move(0);
    right_motors.move(0);
    pros::ADIDigitalOut pistonB('B');
    pistonB.set_value(true);
    intake.move(127);
    pros::delay(2000);
    intake.move(0);
    left_motors.move(127);
    right_motors.move(127);
    pros::delay(500);   // 1000 ms = 1 second
    left_motors.move(0);
    right_motors.move(0);
    outtake.move(127);
    pros::delay(1000);
    outtake.move(0);
}

// ===========================
//  DRIVER CONTROL
// ===========================
void opcontrol() {

    pros::ADIDigitalOut pistonA('A');
    pros::ADIDigitalOut pistonB('B');

    bool pistB = false;
    bool lastA = false;

    const int DEADBAND = 10;  // Helps go straight

    while (true) {

        // ==========================
        // DRIVE
        // ==========================
        int leftY  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Deadband to stop drifting
        if (std::abs(leftY)  < DEADBAND) leftY  = 0;
        if (std::abs(rightX) < DEADBAND) rightX = 0;

        int throttle = leftY;     // forward/back
        int turn     = -rightX;   // left/right turn

        chassis.curvature(throttle, turn);

        // ==========================
        // INTAKE (R2 = forward, R1 = reverse)
        // ==========================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(127);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move(-127);
        }
        else {
            intake.move(0);
        }

        // ==========================
        // OUTTAKE (L2 = forward, L1 = reverse)
        // ==========================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            outtake.move(100);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            outtake.move(-100);
        }
        else {
            outtake.move(0);
        }

        // ==========================
        // PNEUMATIC A (UP/DOWN)
        // ==========================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            pistonA.set_value(true);   // extend
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            pistonA.set_value(false);  // retract
        }

        // ==========================
        // PNEUMATIC B (Button A)
        // ==========================
        bool aNow = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        if (aNow && !lastA) {
            pistB = !pistB;
            pistonB.set_value(pistB);
        }
        lastA = aNow;

        pros::delay(20);
    }
}
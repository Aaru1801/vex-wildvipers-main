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
    // Use lemlib odom starting at "position D"
    // We'll treat that as (0, 0) facing 0 degrees (upfield).
    chassis.setPose(0, 0, 0);

    // PistonB for intake deploy (port B)
    pros::ADIDigitalOut pistonB('B');

    // Make sure everything is stopped at the start
    left_motors.move(0);
    right_motors.move(0);
    intake.move(0);
    outtake.move(0);

    // 1) Drive forward to first point (e.g., ball line)
    //    Target: (0, 24) inches from start
    chassis.moveToPoint(0, 24, 2000);   // 2s timeout

    // 2) Turn 270 degrees to face the balls
    chassis.turnToHeading(270, 2000);   // face left

    // 3) Deploy intake using piston B
    pistonB.set_value(true);   // extend

    // 4) Drive to the balls
    //    Target: (-12, 24) — 12 inches left from where we were
    chassis.moveToPoint(-12, 24, 2000);

    // 5) Run intake to pull in balls while stationary
    intake.move(127);
    pros::delay(1000);         // tune for your ball count
    intake.move(0);

    // 6) Turn to 90 degrees using IMU (face right)
    chassis.turnToHeading(90, 2000);

    // 7) Drive toward goal
    //    Target: (-12, 36) — a bit further upfield
    chassis.moveToPoint(-12, 36, 2000);

    // 8) Outtake to score
    outtake.move(127);
    pros::delay(1000);         // tune how long to outtake
    outtake.move(0);

    // 9) Stop everything at the end
    left_motors.move(0);
    right_motors.move(0);
    intake.move(0);
    outtake.move(0);
}
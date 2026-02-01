#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"

ASSET(right_safe_path_txt);   // name = file name with . replaced by _
extern lemlib::Chassis chassis;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
// LEFT DRIVE = ports 1, 2, 19 (ALL BLUE)
pros::MotorGroup leftMotors({-8, -9, -10}, pros::MotorGearset::blue);

// RIGHT DRIVE = ports 4,5,10 (ALL BLUE)
pros::MotorGroup rightMotors({4, 5, 6}, pros::MotorGearset::blue);

// INTAKE = port 7
pros::Motor intake(1, pros::MotorGearset::blue);

// OUTTAKE = port 8
pros::Motor outtake(11, pros::MotorGearset::blue);

// Inertial Sensor on port 10
pros::Imu imu(13);

// tracking wheels
// vertical tracking wheel encoder. Rotation sensor, port 12, reversed
pros::Rotation verticalEnc(-12);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              15, // 15 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omni
                              600, // drivetrain rpm is 600
                              2 // horizontal drift is 5 cuz of 2 traction wheels. If we had all traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(1.82, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            5, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            3 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(
                                            2,  // kP  (start low)
                                            0.0,  // kI
                                            6.0,  // kD
                                            0,
                                            1.0,  // smallError (deg)
                                            200,  // smallErrorTimeout (ms)
                                            5.0,  // largeError (deg)
                                            600,  // largeErrorTimeout (ms)
                                            3
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x);      // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);      // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    /* CODEGEN EXPORT: LemLib */

    chassis.setPose(-48.000000, 18.000000, 0.000000);

    chassis.moveToPoint(-48.0, 24.0, 552);
    pros::delay(400);
    chassis.moveToPoint(-48.0, 48.0, 1104);
    chassis.waitUntil(8.64);
    intake.move(127);
    pros::ADIDigitalOut pistonA('A');
    pistonA.set_value(true);
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.turnToHeading(270, 4000);
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(200);
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(2000);
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(400);
    leftMotors.move(0);
    rightMotors.move(0);
// Estimated total time: 6.23 s

}

bool pistonAState = false;
bool lastButtonState = false;

/**
 * Runs in driver control
 */
void opcontrol() {
    pros::ADIDigitalOut pistonA('A');
    pros::ADIDigitalOut pistonB('B');

    bool pistA = false;   // state of piston A (UP arrow)
    bool lastUp = false;  // last state of UP button

    bool pistB = false;   // state of piston B (A button)
    bool lastA = false;   // last state of A button

    const int DEADBAND = 10;  // Helps go straight

    while (true) {

        // ==========================================
        // JOYSTICK DRIVE CONTROL
        // ==========================================
        int leftY  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Deadband to stop drifting
        if (std::abs(leftY)  < DEADBAND) leftY  = 0;
        if (std::abs(rightX) < DEADBAND) rightX = 0;

        int throttle = leftY;     // forward/back
        int turn     = rightX;   // left/right turn

        chassis.curvature(throttle, turn);

        // ==========================================
        // INTAKE (R2 = forward, R1 = reverse)
        // ==========================================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move(100);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(-100);
        }
        else {
            intake.move(0);
        }

        // ==========================================
        // OUTTAKE (L2 = forward, L1 = reverse)
        // ==========================================
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            outtake.move(100);
        }
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            outtake.move(-100);
        }
        else {
            outtake.move(0);
        }

        // =============================================
        // LOWER GOAL SCORER PNEUMATIC A (UP ARROW)
        // =============================================
        bool upNow = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        if (upNow && !lastUp) {        // just pressed
            pistA = !pistA;            // toggle state
            pistonA.set_value(pistA);  // apply
        }
        lastUp = upNow;

        // =============================================
        // MATCH LOADER PNEUMATIC B (A BUTTON)
        // =======================================
        bool aNow = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        if (aNow && !lastA) {          // just pressed
            pistB = !pistB;            // toggle state
            pistonB.set_value(pistB);  // apply
        }
        lastA = aNow;

        pros::delay(20);
    }
}
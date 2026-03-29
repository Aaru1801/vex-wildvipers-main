#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <fstream>
#include <algorithm>
#include <sys/_intsup.h>
#include <string>
#include "nlohmann/json.hpp"

// ASSET(right_safe_path_txt);   // name = file name with . replaced by _
extern lemlib::Chassis chassis;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
// LEFT DRIVE = ports 1, 2, 19 (ALL BLUE)
pros::MotorGroup leftMotors({-11, -10, -13}, pros::MotorGearset::blue);

// RIGHT DRIVE = ports 4,5,10 (ALL BLUE)
pros::MotorGroup rightMotors({8, 9, 17}, pros::MotorGearset::blue);

// INTAKE = port 7
pros::Motor intake(1, pros::MotorGearset::blue);

// OUTTAKE = port 8
pros::Motor outtake(15, pros::MotorGearset::blue);

// Inertial Sensor on port 10
pros::Imu imu(19);

// tracking wheels
// vertical tracking wheel encoder. Rotation sensor, port 18
pros::Rotation verticalEnc(-18);
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
lemlib::ControllerSettings linearController(20, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            10, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(
                                            2,  // kP  (start low)
                                            0,  // kI
                                            10,  // kD
                                            0,
                                            0,  // smallError (deg)
                                            0,  // smallErrorTimeout (ms)
                                            0,  // largeError (deg)
                                            0,  // largeErrorTimeout (ms)
                                            20
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tr   acking wheel
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

nlohmann::json mainPath;

namespace {
// JSON path tuning
constexpr double kFieldPx = 600.0;   // JSON coordinate width/height
constexpr double kFieldIn = 72.0;   // field size in inches (12ft)
constexpr double kOriginPx = kFieldPx / 2.0;
constexpr double kInPerPx = kFieldIn / kFieldPx;
constexpr int kDefaultMoveTimeoutMs = 2000;

struct FieldPoint {
    double x;
    double y;
};

bool readPos(const nlohmann::json& pos, FieldPoint& out) {
    if (!pos.is_array() || pos.size() < 2 || !pos[0].is_number() || !pos[1].is_number()) {
        return false;
    }

    const double px = pos[0].get<double>();
    const double py = pos[1].get<double>();

    out.x = (px - kOriginPx) * kInPerPx;
    out.y = (kOriginPx - py) * kInPerPx;
    return true;
}

void runAction(const nlohmann::json& action,
               pros::ADIDigitalOut& pistonA,
               pros::ADIDigitalOut& pistonB,
               pros::ADIDigitalOut& pistonC
               ) {
    if (!action.is_object() || !action.contains("type") || !action["type"].is_string()) {
        return;
    }

    const std::string type = action["type"].get<std::string>();

    if (type == "wait") {
        const double seconds = action.value("s", 0.0);
        if (seconds > 0.0) {
            pros::delay(static_cast<int>(seconds * 1000.0));
        }
        return;
    }

    if (type == "intake") {
        const int power = action.value("power", 127);
        const int ms = action.value("ms", 0);
        intake.move(power);
        if (ms > 0) pros::delay(ms);
        intake.move(0);
        return;
    }

    if (type == "outtake") {
        const int power = action.value("power", 127);
        const int ms = action.value("ms", 0);
        outtake.move(power);
        if (ms > 0) pros::delay(ms);
        outtake.move(0);
        return;
    }

    if (type == "pistonA") {
        const bool value = action.value("value", true);
        pistonA.set_value(value);
        return;
    }

    if (type == "pistonB") {
        const bool value = action.value("value", true);
        pistonB.set_value(value);
        return;
    }

    if (type == "pistonC") {
        const bool value = action.value("value", true);
        pistonC.set_value(value);
        return;
    }
}
} // namespace
void start_autoselector();
void run_selected_auton();

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    // start_autoselector();
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

    std::ifstream mainPathFile("/static/Ninja.json");
    if (!mainPathFile.is_open()) {
        std::cerr << "Error: Could not open the file Ninja.json" << std::endl;
        return;
    }

    // Parse the JSON data from the stream
    try {
        mainPathFile >> mainPath;
    } catch (const std::exception& e) {
        std::cerr << "Error parsing Ninja.json: " << e.what() << std::endl;
    }

    // Close the file
    mainPathFile.close();
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

int savedCount = 0;

void storePosition() {
    const auto pose = chassis.getPose();
    controller.print(0, 0, "X:%.1f Y:%.1f", pose.x, pose.y);
    controller.print(0, 1, "T:%.1f #%d", pose.theta, savedCount + 1);
    savedCount++;
}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void left_autonomous() {
    bool pistonAState = false;
    bool lastButtonState = false;
    pros::ADIDigitalOut pistonA('A'); // de-score mech piston
    pros::ADIDigitalOut pistonB('B'); // middle goal scorer piston
    pros::ADIDigitalOut pistonC('C'); // match loader piston


    chassis.setPose(0,0,0);
    pros::delay(100);
    chassis.moveToPose(0, 39, 0, 1200, {.maxSpeed = 100, .minSpeed = 80});
    chassis.waitUntilDone();
    // align to matchloader
    chassis.turnToHeading(270, 500);
    pros::delay(200);
    pistonC.set_value(true);
    pros::delay(200);
    // move to left matchloader
    chassis.moveToPoint(-17, 39, 1050, {.maxSpeed = 80});
    chassis.waitUntilDone();
    intake.move(127);
    pros::delay(1500);
    chassis.waitUntilDone();
    // move to left long goal
    chassis.moveToPoint(26, 39, 1500, {.forwards = false, .maxSpeed = 100, .minSpeed = 90});
    chassis.waitUntilDone();
    // outtake the loads
    outtake.move(127);
    pros::delay(2000);
    intake.move(0);
    outtake.move(0);

    chassis.moveToPoint(0, 39, 700);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.turnToHeading(0, 500);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.moveToPoint(0, 17, 1200, {.forwards = false, .maxSpeed = 100, .minSpeed = 90});
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.turnToHeading(90, 500);
    chassis.waitUntilDone();
    pros::delay(100);
    intake.move(127);
    chassis.moveToPoint(30, 17, 1200);
    chassis.waitUntilDone();
}

void right_autonomous() {
    bool pistonAState = false;
    bool lastButtonState = false;
    pros::ADIDigitalOut pistonA('A'); // de-score mech piston
    pros::ADIDigitalOut pistonB('B'); // middle goal scorer piston
    pros::ADIDigitalOut pistonC('C'); // match loader piston


    chassis.setPose(0,0,0);
    pros::delay(100);
    chassis.moveToPose(0, 42, 0, 1200, {.maxSpeed = 100, .minSpeed = 80});
    chassis.waitUntilDone();
    // align to matchloader
    chassis.turnToHeading(90, 1200);
    pros::delay(500);
    pistonC.set_value(true);
    pros::delay(500);
    // move to right matchloader
    chassis.moveToPoint(20, 42, 1100, {.maxSpeed = 80});
    chassis.waitUntilDone();
    // intake the loads
    intake.move(127);
    pros::delay(2400);
    // move to right long goal
    chassis.moveToPoint(-26, 40.3, 1200, {.forwards = false, .maxSpeed = 100, .minSpeed = 50});
    chassis.waitUntilDone();
    // outtake the loads
    outtake.move(127);
    pros::delay(3600);
    intake.move(0);
    outtake.move(0);
    pros::delay(500);
    chassis.moveToPoint(0, 42, 1200, {.forwards = false, .maxSpeed = 100, .minSpeed = 80});
    chassis.waitUntilDone();
}

void test_autonomous() {
    // example of a simple autonomous routine that just moves the robot forward
    chassis.setPose(0,0,0);
    chassis.turnToHeading(90, 500);
    chassis.waitUntilDone();
}


void skills_autonomous() {
    // Path
bool pistonAState = false;
bool lastButtonState = false;
pros::ADIDigitalOut pistonA('A'); // de-score mech piston
pros::ADIDigitalOut pistonB('B'); // middle goal scorer piston
pros::ADIDigitalOut pistonC('C'); // match loader piston

// ==========================
// RIGHT SIDE STARTS HERE!!!!!
// ==========================
    chassis.setPose(0,0,0);
    pros::delay(250);
    chassis.moveToPoint(0, 42, 1200, {.maxSpeed = 100});
    chassis.waitUntilDone();
    pros::delay(150);
    // align to matchloader
    chassis.turnToPoint(17, 42, 1100);
    chassis.waitUntilDone();
    pros::delay(500);
    pistonC.set_value(true);
    pros::delay(500);
    intake.move(127);
    // move to right matchloader
    chassis.moveToPoint(17, 42, 1100, {.maxSpeed = 90});
    chassis.waitUntilDone();
    pros::delay(2500);
    intake.move(0);
    // move to midway of right matchloader and right long goal
    chassis.moveToPoint(0, 42, 1000, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(150);
    pistonC.set_value(false);
    pros::delay(150);
    chassis.turnToHeading(0, 400);
    chassis.waitUntilDone();
    // move to wall beside of right long goal
    chassis.moveToPoint(0, 57.5, 1200, {.maxSpeed = 100, .minSpeed = 80});
    chassis.waitUntilDone();
    // turn to face opposite side beside the right long goal
    chassis.turnToPoint(-93.1, 55.5, 500, {.forwards = false});
    chassis.waitUntilDone();
    // move to opposite end of right matchloader
    chassis.moveToPoint(-99.1, 56.5, 1500, {.forwards = false, .maxSpeed = 100, .minSpeed = 80});
    chassis.waitUntilDone();

    // turn to face mid way of opposite side right matchloader and right long goal
    chassis.turnToPoint(-92.5, 42, 500, {.forwards = false});
    chassis.waitUntilDone();

    // move midway to opposite end of right matchloader and right long goal
    chassis.moveToPoint(-99.1, 41, 1600, {.forwards = false});
    chassis.waitUntilDone();
    // turn to face opposite side right long goal
    chassis.turnToPoint(-85.4, 37, 600, {.forwards = false});
    chassis.waitUntilDone();
    // move to opposite end of right long goal and score previously intaked loads
    chassis.moveToPoint(-85.4, 37, 1000, {.forwards = false});
    chassis.waitUntilDone();
    // outtake the loads
    intake.move(127);
    outtake.move(127);
    pros::delay(2550);
    outtake.move(0);
    intake.move(0);
    // put matchloader piston down to intake next loads
    pros::delay(350);
    pistonC.set_value(true);
    pros::delay(350);
    intake.move(127);
    // move to opposite end matchloader
    chassis.moveToPoint(-114, 38, 1500);
    chassis.waitUntilDone();
    pros::delay(2450);
    intake.move(0);
    // move to opposite end of right long goal and score intaked loads
    chassis.moveToPoint(-85.4, 37, 1450, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(450);
    // outtake the loads
    intake.move(127);
    outtake.move(127);
    pros::delay(2550);
    outtake.move(0);
    intake.move(0);
    pros::delay(150);
    pistonC.set_value(false);
    // move back to midway of opposite side right matchloader and right long goal
    chassis.moveToPoint(-91, 38, 1600);
    chassis.waitUntilDone();

// ==========================
// LEFT SIDE STARTS HERE!!!!!
// ==========================
    // turn to face opposite side left long goal
    chassis.turnToPoint(-91, -69, 500, {.forwards = false});
    chassis.waitUntilDone();
    // move midway to the opposite side left long goal
    chassis.moveToPoint(-91, -69, 1600, {.forwards = false});
    chassis.waitUntilDone();
    // turn to front face the opposite side left match loader
    chassis.turnToHeading(270, 600);
    chassis.waitUntilDone();
    // put down piston to intake loads
    pros::delay(350);
    pistonC.set_value(true);
    pros::delay(350);
    // go to opposite side left matchloader
    chassis.moveToPoint(-113, -67, 1600);
    chassis.waitUntilDone();
    // intake the loads
    intake.move(127);
    pros::delay(2550);
    intake.move(0);
    pros::delay(150);
    // move to midway to opposite side of left matchloader and left long goal
    chassis.moveToPoint(-91, -67, 1600, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(150);
    pistonC.set_value(false);
    // turn to face the left wall
    chassis.turnToPoint(-91, -81, 500);
    chassis.waitUntilDone();
    pros::delay(350);
    // go to the left wall
    chassis.moveToPoint(-91, -81, 1250);
    chassis.waitUntilDone();
    // turn to face beside left long goal
    chassis.turnToPoint(-3, -79, 500, {.forwards = false});
    chassis.waitUntilDone();
    // move to beside left long goal
    chassis.moveToPoint(-3, -79, 1600, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(350);
    // turn to midway to left long goal and left matchloader
    chassis.turnToPoint(-3, -65, 500, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(350);
    // move to midway to left long goal and left matchloader
    chassis.moveToPoint(-3, -65, 1600, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(350);
    // outtake the loads
    intake.move(127);
    outtake.move(127);
    pros::delay(2500);
    outtake.move(0);
    intake.move(0);
    // put piston down to intake next loads
    pros::delay(350);
    pistonC.set_value(true);
    pros::delay(350);
    // move to left matchloader
    intake.move(127);
    chassis.moveToPoint(17, -64, 1200);
    chassis.waitUntilDone();
    pros::delay(2350);
    intake.move(0);
    pros::delay(150);
    // move to left long goal and score the loads
    chassis.moveToPoint(-3, -65, 1200, {.forwards = false});
    chassis.waitUntilDone();
    // outtake the loads
    intake.move(127);
    outtake.move(127);
    pros::delay(2500);
    outtake.move(0);
    intake.move(0);
    pros::delay(350);
    // move to midway of left long goal and left matchloader
    chassis.moveToPoint(1, -65, 1600);
    chassis.waitUntilDone();
    // turn to face the turn before park zone
    chassis.turnToPoint(16, -20, 500, {.forwards = false});
    chassis.waitUntilDone();
    // move to the turn before park zone
    chassis.moveToPoint(16, -20, 1200, {.forwards = false});
    chassis.waitUntilDone();
    // move to park zone
    chassis.moveToPoint(16, -5, 1200, {.forwards =  false});
    chassis.waitUntilDone();
}

void solo_awp_autonomous() {
    // example of a simple autonomous routine that just moves the robot forward
    // TO BE DONE: replace with your actual AWP autonomous
}

void autonomous() {
    skills_autonomous();
}

bool pistonAState = false;
bool lastButtonState = false;

/**
 * Runs in driver control
 */
void opcontrol() {
    pros::ADIDigitalOut pistonA('A');
    pros::ADIDigitalOut pistonB('B');
    pros::ADIDigitalOut pistonC('C');

    bool pistA = false;   // state of piston A (UP arrow)
    bool lastUp = false;  // last state of UP button

    bool pistB = false;   // state of piston B (L1 hold)
    bool lastA = false;   // last state of A button

    bool pistC = false;   // state of piston C (DOWN arrow)
    bool lastDown = false; // last state of DOWN button

    bool lastX = false;   // last state of X button (save coordinates)

    const int DEADBAND = 10;  // Helps go straight

    while (true) {

        // ==========================================
        // JOYSTICK DRIVE CONTROL
        // ==========================================
        int forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn    = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        if (std::abs(forward) < DEADBAND) forward = 0;
        if (std::abs(turn) < DEADBAND) turn = 0;

        int leftPower  = forward + turn;
        int rightPower = forward - turn;

        leftPower  = std::max(-127, std::min(127, leftPower));
        rightPower = std::max(-127, std::min(127, rightPower));

        leftMotors.move(leftPower);
        rightMotors.move(rightPower);

        int intakePower = 0;
        int outtakePower = 0;

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intakePower = 127;
            outtakePower = 127;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intakePower = 127;
            outtakePower = -127;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intakePower = 127;
            outtakePower = 0;
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intakePower = -127;
            outtakePower = 0;
        }

        intake.move(intakePower);
        outtake.move(outtakePower);

        // =============================================
        // MIDDLE GOAL SCORER PNEUMATIC B (L1 HOLD)
        // =============================================
        bool l1Now = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        pistB = l1Now;
        pistonB.set_value(pistB);

        // =============================================
        // DE-SCORE MECH PNEUMATIC B (B BUTTON)
        // =======================================
        bool aNow = controller.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        if (aNow && !lastA) {          // just pressed
            pistA = !pistA;            // toggle state
            pistonA.set_value(pistA);  // apply
        }
        lastA = aNow;

        // =============================================
        // MATCHLOADER PNEUMATIC C (A BUTTON)
        // =======================================
        bool bNow = controller.get_digital(pros::E_CONTROLLER_DIGITAL_A);
        if (bNow && !lastDown) {        // just pressed
            pistC = !pistC;                // toggle state
            pistonC.set_value(pistC);      // apply
        }
        lastDown = bNow;

        // =============================================
        // SAVE COORDINATES (X BUTTON)
        // =============================================
        bool xNow = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        if (xNow && !lastX) {          // just pressed (rising edge)
            storePosition();
        }
        lastX = xNow;

        pros::delay(20);
    }
}
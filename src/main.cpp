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
// vertical tracking wheel encoder. Rotation sensor, port 12, reversed
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
    chassis.turnToHeading(270, 1200);
    pros::delay(500);
    pistonC.set_value(true);
    pros::delay(500);
    // move to left matchloader
    chassis.moveToPoint(-14, 39, 1200, {.maxSpeed = 80});
    chassis.waitUntilDone();
    intake.move(127);
    pros::delay(2400);
    chassis.waitUntilDone();
    // move to left long goal
    chassis.moveToPoint(26, 39, 1500, {.forwards = false, .maxSpeed = 100, .minSpeed = 50});
    chassis.waitUntilDone();
    // outtake the loads
    outtake.move(127);
    pros::delay(2400);
    intake.move(0);
    outtake.move(0);
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

void skills_autonomous() {
    bool pistonAState = false;
    bool lastButtonState = false;
    pros::ADIDigitalOut pistonA('A'); // de-score mech piston
    pros::ADIDigitalOut pistonB('B'); // middle goal scorer piston
    pros::ADIDigitalOut pistonC('C'); // match loader piston


    chassis.setPose(0,0,0);
    pros::delay(150);
    chassis.moveToPose(0, 42, 0, 1200, {.maxSpeed = 100, .minSpeed = 80});
    chassis.waitUntilDone();
    // align to matchloader
    chassis.turnToHeading(90, 700);
    pros::delay(600);
    pistonC.set_value(true);
    pros::delay(600);
    // move to right matchloader
    chassis.moveToPoint(18, 42, 1100, {.maxSpeed = 80});
    chassis.waitUntilDone();
    // intake the loads
    intake.move(127);
    pros::delay(2400);
    intake.move(40);
    
    // move partway to right long goal
    chassis.moveToPoint(0, 42, 1200, {.forwards = false, .maxSpeed = 100, .minSpeed = 50});
    chassis.waitUntilDone();
    pros::delay(600);
    // turn to face the wall
    chassis.turnToHeading(0, 700);
    chassis.waitUntilDone();
    pros::delay(600);
    // move to the wall
    chassis.moveToPoint(0, 52, 1200, {.forwards = false, .maxSpeed = 100, .minSpeed = 50});
    chassis.waitUntilDone();
    pros::delay(600);

    // turn to move straight to the opposite end of the field
    chassis.turnToHeading(90, 700);
    chassis.waitUntilDone();
    pros::delay(600);
    // move to opposite end of the field
    chassis.moveToPoint(-103, 52, 1200, {.forwards = false, .maxSpeed = 100, .minSpeed = 50});
    chassis.waitUntilDone();
    pros::delay(600);
    // turn to go straight to turn to the opposite matchloader
    chassis.turnToHeading(0, 700);
    chassis.waitUntilDone();
    pros::delay(600);
    // move to partway of opposite matchloader to outtake the loads from first matchloader in the right long goal
    chassis.moveToPoint(-103, 42, 1200, {.maxSpeed = 100, .minSpeed = 50});
    chassis.waitUntilDone();
    pros::delay(600);

    // face the long goal to outtake the loads from the first matchloader
    chassis.turnToHeading(270, 700);
    chassis.waitUntilDone();
    pros::delay(600);

    // outtake the loads form previous matchloader in the right long goal from opposite end
    chassis.moveToPoint(-93, 42, 1200, {.forwards = false, .maxSpeed = 100, .minSpeed = 50});
    chassis.waitUntilDone();
    outtake.move(127);
    pros::delay(2400);
    outtake.move(0);
    pros::delay(600);

    // intake the loads from opposite match loader
    intake.move(127);
    chassis.moveToPoint(-113, 42, 1200, {.maxSpeed = 100, .minSpeed = 50});
    chassis.waitUntilDone();
    intake.move(40);

    // outtake the loads in the right long goal
    chassis.moveToPoint(-93, 42, 1200, {.forwards = false, .maxSpeed = 100, .minSpeed = 50});
    chassis.waitUntilDone();
    pros::delay(600);
    // score the loads in the opposite end of right long goal
    intake.move(127);
    outtake.move(127);
    pros::delay(3600);
    intake.move(0);
    outtake.move(0);
    pros::delay(500);

    // move mid of match loader and long goal to turn to clear red park zone
    chassis.moveToPoint(-103, 42, 1200, {.forwards = false, .maxSpeed = 100, .minSpeed = 50});
    chassis.waitUntilDone();
    pros::delay(600);

    // turn to clear red park zone
    chassis.turnToHeading(0, 700);
    chassis.waitUntilDone();
    pros::delay(600);

    // move a few inches to turn and after clear red park zone
    chassis.moveToPoint(-103, 26, 1200, {.forwards = false, .maxSpeed = 100, .minSpeed = 50});
    chassis.waitUntilDone();
    pros::delay(600);
}

void solo_awp_autonomous() {
    // example of a simple autonomous routine that just moves the robot forward
    // TO BE DONE: replace with your actual AWP autonomous
}

void autonomous() {
    right_autonomous();
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

    bool pistB = false;   // state of piston B (A button)
    bool lastA = false;   // last state of A button

    bool pistC = false;   // state of piston C (DOWN arrow)
    bool lastDown = false; // last state of DOWN button

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
        // MIDDLE GOAL SCORER PNEUMATIC A (UP ARROW)
        // =============================================
        bool upNow = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        if (upNow && !lastUp) {        // just pressed
            pistB = !pistB;            // toggle state
            pistonB.set_value(pistB);  // apply
        }
        lastUp = upNow;

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

        pros::delay(20);
    }
}
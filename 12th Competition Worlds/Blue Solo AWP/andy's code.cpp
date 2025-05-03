#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include <sys/signal.h>

pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Drive train
pros::Motor Intake(-4, pros::MotorGearset::blue);
pros::MotorGroup Ldrive({17,-14 ,-13}, pros::MotorGearset::blue);
pros::MotorGroup Rdrive({-19,16,11}, pros::MotorGearset::blue);
pros::Motor ArmL (-15,pros::MotorGearset::green);
pros::Motor ArmR (9, pros::MotorGearset::green);
pros::MotorGroup Arm({-15, 9}, pros::MotorGearset::green);
pros::Rotation Arm_Angle(7);
pros::adi::DigitalOut Clamp('A');
pros::adi::DigitalOut LeftDoinker('B');
pros::adi::DigitalOut GoalTipper('C');
pros::adi::DigitalOut RightDoinker('D');
pros::adi::DigitalIn Eject('E');
pros::Optical Color(3);

//Inertial senser
pros::Imu Auton(21);

// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 10, reversed
pros::Rotation horizontalEnc(-10);
// vertical tracking wheel encoder. Rotation sensor, port 5, reversed
pros::Rotation verticalEnc(-5);
// horizontal tracking wheel. 2" diameter, 0" offset, middle of the robot (pos)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 0);
// vertical tracking wheel. 2" diameter, 0.5" offset, right of the robot (pos)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, 0.5);


// drivetrain settings
lemlib::Drivetrain drivetrain(&Ldrive, // left motor group
                              &Rdrive, // right motor group
                              10.5, // 10.5 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);


// lateral motion controller
lemlib::ControllerSettings linearController(14, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            110, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);


// angular motion controller
lemlib::ControllerSettings angularController(5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             40, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                            3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);


// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &Auton // inertial sensor
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

void screen() {
    // loop forever
    while (true) {
    
         pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
    }
}

void initialize() {


    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms


    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs


    // thread to for brain screen and position logging
    pros::lcd::initialize();
    chassis.calibrate();
    //color_sensor.set_integration_time(5);
	pros::Task screenTask(screen);
    Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Color.set_integration_time(5);
    Color.set_led_pwm(100);
    Arm_Angle.set_position(1 * 100);
	Intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    int joystick_value = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    pros::lcd::print(4, "Joystick Valye: %d", joystick_value); 
    // pros::Task screenTask([&]() {
    //     while (true) {
    //         // print robot location to the brain screen
    //         pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    //         pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    //         pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    //         // log position telemetry
    //         lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
    //         // delay to save resources
    //         pros::delay(50);
    //     }
    // });
}
// Enum for color detection
enum Color {RED, BLUE, UNKNOWN};
int intakeSpeed = 127; // Default intake speed

enum Color readColor() {
    int hue = Color.get_hue();
    if (hue > 0 && hue < 20) {
        return RED;
    } else if (hue > 200 && hue < 360) {
        return BLUE; 
    }
    return UNKNOWN;
}

void RedRing() {
    // Add RedRing logic here if needed
}

void BlueRing() {
    int originalSpeed = intakeSpeed; // Store the original intake speed

    // Wait until line sensor value is under 3000 before ejecting
    while (Eject.get_value() >= 2800) {
        pros::delay(2);
    }
    pros::delay(20);
    

    // Reduce speed temporarily
    intakeSpeed *= 0.85;
    Intake.move(-intakeSpeed);

    pros::delay(50);

    // Restore the original intake speed
    intakeSpeed = originalSpeed;
    Intake.move(intakeSpeed);
}

pros::Task* colorSorter = nullptr;

void autoEject() {
    while (true) {
        enum Color detectedColor = readColor();
        
        if (detectedColor == RED) {
            RedRing();
        } else if (detectedColor == BLUE) {
            BlueRing();
        }
        
        pros::delay(10);
    }
}



// const double kP = 2;
double armTarget = 0;
bool toggleArmPosition = false;
bool wall_stake = false;
bool alliance_stake = false;


// void updateArm() {
//     while (true) {
//         double currentAngle = Arm_Angle.get_position() / 100.0;
//         double error = armTarget - currentAngle;
//         double power = kP * error;
//         Arm.move(power);
//         pros::delay(20);
//     }
// }
const double kP = 1.49;  // Reduced P for smoother moves
const double kI = 0.001; // Tiny I to avoid windup
const double kD = 3;  // Strong D to kill oscillations

const double maxPower = 127.0;
const double minPower = 10.0; 
const double deadband = 2.0;  
const double integralLimit = 15; // Lower limit for less windup

double lastError = 0;
double integral = 0;

void updateArm() {
    Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // Stop arm from "drifting"

    while (true) {
        double currentAngle = Arm_Angle.get_position() / 100.0;
        double error = armTarget - currentAngle;

        // Reset integral if close enough to target
        if (fabs(error) < deadband) {
            integral = 0;
        } else {
            integral += error;
            integral = std::clamp(integral, -integralLimit, integralLimit);
        }

        // Derivative (with smoothing for less sudden changes)
        double derivative = (error - lastError) * 0.9;

        // PID calculation
        double power = (kP * error) + (kI * integral) + (kD * derivative);

        // Clamp power
        power = std::clamp(power, -maxPower, maxPower);

        // Soft stop — slower as it gets close to the target
        if (fabs(error) < deadband) {
            Arm.move(0); 
        } else if (fabs(error) < 10) {  // Gradually slow down near target
            Arm.move(power * 0.9);  
        } else {
            Arm.move(power);
        }

        lastError = error;
        pros::delay(20);
    }
}











/**
 * Runs while the robot is disabled
 */
void disabled() {}


/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

ASSET(example_txt);


/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    pros::Task armPID(updateArm);
    if (colorSorter == nullptr) {
        colorSorter = new pros::Task(autoEject);
    }
    
    chassis.setPose(-1.67, 1.26, -23.9);
    chassis.moveToPoint(-17, 45, 1000, {.minSpeed = 120});
    LeftDoinker.set_value(true);
    // chassis.waitUntil(48);
    Intake.move(127);
    chassis.waitUntilDone();
    pros::delay(100);
    chassis.moveToPoint(4.3, 28.53, 1200, {.forwards = false, .maxSpeed = 70});
    pros::delay(600);
    Intake.move(0);
    chassis.waitUntil(20);
    Clamp.set_value(true);
    pros::delay(80);
    Intake.move(127);
    chassis.waitUntilDone();
    LeftDoinker.set_value(false);
    chassis.moveToPoint(-17.6, 36.62, 500, {.minSpeed = 60, .earlyExitRange = 1});
    chassis.moveToPoint(-23, 34, 500);
    chassis.turnToPoint(-41.5, -7, 600);
    chassis.moveToPoint(-43.5, -7, 2000, {.maxSpeed = 110});
    chassis.waitUntilDone();
    pros::delay(300);
    chassis.moveToPoint(-44, 4.67, 500, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(300);
    chassis.turnToPoint(18.212568, 1.74, 800);
    chassis.moveToPoint(10.21, 0, 1000, {.maxSpeed = 100, .minSpeed = 60, .earlyExitRange = 3});
    chassis.moveToPoint(18.21, 0, 1000, {.maxSpeed = 601});



    


}


bool autoEjectEnabled = false;  // Toggle state for auto-eject

void opcontrol() {
    bool Intake_piston = false;
    int arm_speed = 0;
    int arm_angle = 0;
    int intake_speed = 0;
    bool clamp_down = false;
    bool Goaltip = false;
    armTarget = 1;
    pros::Task armControl(updateArm);

    while (true) {
        arm_angle = Arm_Angle.get_position() / 100;
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

        // Toggle Auto-Eject when 'B' is pressed
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            autoEjectEnabled = !autoEjectEnabled;
        }

        if (autoEjectEnabled) {
            // Auto-eject mode is ON, but still allow manual intake control
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intake_speed = -127;
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake_speed = 127;
            } else {
                intake_speed = 127;  // Default to 127 in auto-eject mode
            }

            // Check colors and eject accordingly
            enum Color detectedColor = readColor();
            if (detectedColor == RED) {
                RedRing();
            } else if (detectedColor == BLUE) {
                BlueRing();
            }

        } else {
            // Auto-eject is OFF, normal intake control
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intake_speed = -127;
            } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake_speed = 127;
            } else {
                intake_speed = 0;
            }
        }

        // Apply intake speed to motor
        Intake.move(intake_speed);

        // Clamp
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            clamp_down = !clamp_down;
            Clamp.set_value(clamp_down);
        }

        // Goal Tipper
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            Goaltip = !Goaltip;
            GoalTipper.set_value(Goaltip);
        }

        // Doinker
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            RightDoinker.set_value(true);
            LeftDoinker.set_value(true);
        } else {
            RightDoinker.set_value(false);
            LeftDoinker.set_value(false);
        }

        // Arm Control
        arm_angle = Arm_Angle.get_position() / 100.0;

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            // alliance_stake = !alliance_stake;
            wall_stake = true;
            armTarget = 192;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            wall_stake = false;
            toggleArmPosition = !toggleArmPosition;
            armTarget = toggleArmPosition ? 30 : 70;
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            wall_stake = !wall_stake;
            toggleArmPosition = false;
            armTarget = wall_stake ? 158 : 1;
            Intake.move(toggleArmPosition ? -10 : 0);
        }

        pros::delay(10);  // Save CPU resources
    }
}
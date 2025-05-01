/* -------------------------------- Includes -------------------------------- */
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include <cstddef>
#include <cstdio>
#include <iostream>
#include "main.h"

/* ------------------------------- Controller ------------------------------- */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* --------------------------------- Motors --------------------------------- */
pros::MotorGroup left_drive({-3, -11, 12}, pros::MotorGearset::blue);
pros::MotorGroup right_drive({4, 2, -1}, pros::MotorGearset::blue);

pros::Motor intake(-13, pros::MotorGearset::blue);
pros::MotorGroup arm({-6, 7}, pros::MotorGearset::red);

/* --------------------------------- Sensors -------------------------------- */
pros::Rotation arm_rotation(5);
pros::Rotation vert(-14);
pros::Optical color_sensor(19);
pros::IMU inertial(20);

/* --------------------------------- Pistons -------------------------------- */
pros::adi::DigitalOut intake_lift('B');
pros::adi::DigitalOut clamp('C');
pros::adi::DigitalOut sweeper('A');

/* ----------------------------- Tracking Wheels ---------------------------- */
lemlib::TrackingWheel vert_wheel(&vert, lemlib::Omniwheel::NEW_2, 0.5);

/* ---------------------------- Drivetrain Setup ---------------------------- */
lemlib::Drivetrain drivetrain(
	&left_drive,
    &right_drive,
    10.375,
    lemlib::Omniwheel::NEW_275,
    450,
    2
);

lemlib::OdomSensors sensors(
	&vert_wheel,
    nullptr,
    nullptr,
    nullptr,
    &inertial
);

/* ---------------------------------- PIDs ---------------------------------- */
lemlib::ControllerSettings lateral_controller(
    7.5,
    0,
    10,
    3,
    1,
    100,
    3,
    500,
    10
);

lemlib::ControllerSettings angular_controller(
    2.15,
    0,
    22,
    3,
    1,
    100,
    3,
    500,
    7.6
);

/* ----------------------------- Create Chassis ----------------------------- */
lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    sensors
);

/* -------------------------------- Global Variables ------------------------------- */
bool clamp_down = false;
bool sweeper_down = false;

// Sort modes can be "red", "blue", or "none"
std::string detected_color = "none";
std::string color_to_eject = "blue";

/* ---------------------------- Custom Functions ---------------------------- */
void armTo(int target, int timeout = 1000, float arm_kP = 0.5, int max_speed = 127, int min_speed = 8) {
    int arm_speed;
    int arm_range = 1;
    int arm_range_timeout = 100;

    float arm_angle;
    float arm_difference;

    bool running = true;
    bool in_range = false;
    int current_time = 0;
    int start_time = pros::c::millis();

    while (running) {
        // Set variables
        arm_angle = arm_rotation.get_position() / 100.0;
        arm_difference = target - arm_angle;
        arm_speed = arm_kP * arm_difference;

        // Check minimum speed
        if (arm_speed < min_speed && arm_speed > 0) {
            arm_speed = min_speed;
        } else if (arm_speed > -min_speed && arm_speed < 0) {
            arm_speed = -min_speed;
        }

        // Check maximum speed
        if (arm_speed > 0 && arm_speed > max_speed) {
            arm_speed = max_speed;
        } else if (arm_speed < 0 && arm_speed < -max_speed) {
            arm_speed = -max_speed;
        }

        // Check if done
        if (abs(arm_difference) <= arm_range) {
            if (current_time > 0) {
                if (pros::c::millis() - current_time >= arm_range_timeout) {
                    running = false;
                }

            } else {
                current_time = pros::c::millis();
            }

        } else {
            current_time = 0;
        }

        // Check if timed out
        if (pros::c::millis() - start_time >= timeout) {
            running = false;
        }

        arm.move(arm_speed);

        pros::delay(20);
    }
    arm.brake();
}

int hue;
int proximity;

void checkColor() {
    hue = color_sensor.get_hue();
    proximity = color_sensor.get_proximity();

    if (hue >= 0 && hue <= 30) {
        detected_color = "red";

    } else if (hue >= 200 && hue <= 250) {
        detected_color = "blue";

    } else {
        detected_color = "none";
    }
}

void colorSort() {
    while (true) {
        checkColor();

        if (detected_color == color_to_eject) {
            pros::delay(90);
            intake.move(-70);
            pros::delay(250);
        } else {
            intake.move(120);
        }

        pros::delay(10);
    }
}

// Set color sorting task to nothing
pros::Task* colorSorter = nullptr;

void initialize() {
	/* ----------------------------- Motor Stopping ----------------------------- */
	left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

    /* ---------------------------------- Setup --------------------------------- */
    pros::lcd::initialize();
    chassis.calibrate();

    arm_rotation.reset_position();
    arm_rotation.set_position(0);

    color_sensor.set_led_pwm(100);

    /* -------------------------- Display Data on Brain ------------------------- */
    int arm_angle;

    pros::Task screen_task([&]() {

        while (true) {
            arm_angle = arm_rotation.get_position() / 100;

            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(3, "Arm Angle: %d", arm_angle);

            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // Setup
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    arm_rotation.set_position(0 * 100);

    // Alliance Stake
    intake.move(127);
    pros::delay(300);
    intake.brake();

    // 1st Goal
    chassis.moveToPoint(0, 16, 800);

    chassis.turnToPoint(19, 15.5, 800, {.forwards = false});
    chassis.moveToPoint(19, 15.5, 1000, {.forwards = false, .maxSpeed = 70});

    chassis.waitUntilDone();
    clamp.set_value(true);

    // 1st Ring
    pros::delay(200);
    chassis.turnToHeading(0, 800);
    chassis.moveToPose(22.8, 32.5, 30, 1000, {
        .minSpeed = 120,
        .earlyExitRange = 4
    });
    intake.move(127);

    // Wall Stake Ring
    chassis.moveToPoint(49, 85, 1500);

    // Wall Stake
    chassis.turnToPoint(46.5, 65, 1000);
    armTo(56);
    chassis.moveToPoint(46.5, 65, 1000);

    chassis.turnToPoint(66, 66, 800);
    chassis.waitUntilDone();
    pros::delay(200);

    intake.brake();
    armTo(200);
    intake.move(127);

    chassis.moveToPoint(66, 66, 1200);
    chassis.waitUntilDone();
    armTo(470, 1000, 0.7, 127, 8);

    // 3 Rings in a Line
    chassis.moveToPoint(52.5, 66, 800, {.forwards = false});

    chassis.turnToPoint(52.5, 2.5, 800);
    chassis.moveToPoint(52.5, 2.5, 2000, {.maxSpeed = 80});
    armTo(2);

    // 6th Ring
    chassis.moveToPoint(52, 6, 800, {.forwards = false});

    chassis.turnToPoint(67, 16.5, 800);
    chassis.moveToPoint(67, 16.5, 800);

    // Place 1st Goal in Corner
    chassis.turnToPoint(68, 6, 800, {.forwards = false});
    chassis.moveToPoint(68, 6, 800, {.forwards = false});

    chassis.waitUntilDone();
    pros::delay(200);
    intake.brake();
    clamp.set_value(false);

    // 2nd Goal
    chassis.moveToPoint(65, 14.5, 800);

    chassis.turnToPoint(-6, 12, 800, {.forwards = false});
    chassis.moveToPoint(-6, 12, 2000, {
        .forwards = false,
        .minSpeed = 60,
        .earlyExitRange = 10
    });
    chassis.moveToPoint(-16, 12, 1200, {.forwards = false, .maxSpeed = 60});

    chassis.waitUntilDone();
    clamp.set_value(true);

    // 1st Ring
    pros::delay(200);
    chassis.turnToHeading(0, 800);

    chassis.moveToPose(-24, 38.5, -30, 1000, {
        .minSpeed = 120,
        .earlyExitRange = 4
    });
    intake.move(127);

    // 2nd Wallstake Ring
    chassis.moveToPoint(-44.5, 81, 1500);

    // 2nd Wall Stake
    chassis.turnToPoint(-45, 62.5, 800);
    armTo(56);
    chassis.moveToPoint(-45, 62.5, 1100);

    chassis.turnToPoint(-62, 62, 800);
    chassis.waitUntilDone();
    pros::delay(200);

    intake.brake();
    armTo(200);
    intake.move(127);

    chassis.moveToPoint(-62, 62, 1200);
    chassis.waitUntilDone();
    armTo(470, 1000, 0.7, 127, 8);

    // 3 Rings in a Line
    chassis.moveToPoint(-47.5, 62, 800, {.forwards = false});
    
    chassis.turnToPoint(-48.5, -2, 800);
    chassis.moveToPoint(-48.5, -2, 2000, {.maxSpeed = 80});
    armTo(2);

    // 6th Ring
    chassis.turnToPoint(-63.5, 12, 800);
    chassis.moveToPoint(-63.5, 12, 1000);

    // Place 2nd Goal in Corner
    chassis.turnToPoint(-66, 2.5, 800, {.forwards = false});
    chassis.moveToPoint(-66, 2.5, 800, {.forwards = false});

    chassis.waitUntilDone();
    intake.move(-40);
    clamp.set_value(false);

    // 1st Ring
    chassis.moveToPoint(-28, 81.5, 2000);
    chassis.waitUntil(10);
    intake.move(127);
    armTo(56);

    // 3rd Goal in Corner
    chassis.turnToPoint(-23.5, 116, 800);
    chassis.moveToPoint(-23.5, 116, 1200);

    chassis.swingToHeading(-64, lemlib::DriveSide::LEFT, 1200);
    chassis.moveToPoint(-53, 127.5, 1000);

    // 4th Goal
    chassis.turnToPoint(1, 111, 800, {.forwards = false});
    chassis.moveToPoint(1, 111, 2500, {.forwards = false, .maxSpeed = 60});

    chassis.waitUntilDone();
    clamp.set_value(true);

    // Blue Alliance Stake
    pros::delay(200);
    chassis.turnToHeading(0, 800);
    chassis.moveToPoint(1, 127, 900);
    chassis.moveToPoint(1, 120, 800, {.forwards = false});
    chassis.waitUntilDone();
    intake.brake();
    pros::delay(200);
    armTo(685);
    chassis.moveToPoint(1, 100, 1000, {.forwards = false});

    // For Testing
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    intake.brake();
}

void opcontrol() {
	/* -------------------------------- Variables ------------------------------- */
	int dead_zone = 8;
	int intake_speed = 0;
    bool automatic_intake = false;

    int arm_mode = 0;
    int arm_target;
    int arm_speed;
    int min_arm_speed = 8;

    float arm_angle = 0;
    float arm_kP = 0.6;
    float arm_kD = 15;
    float arm_difference;

    // Setup
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    arm_rotation.set_position(0 * 100);

    // Alliance Stake
    intake.move(127);
    pros::delay(300);
    intake.brake();

    // 1st Goal
    chassis.moveToPoint(0, 16, 800);

    chassis.turnToPoint(19, 15.5, 800, {.forwards = false});
    chassis.moveToPoint(19, 15.5, 1000, {.forwards = false, .maxSpeed = 70});

    chassis.waitUntilDone();
    clamp.set_value(true);

    clamp_down = true;

    // loop forever
    while (true) {
        /* --------------------------- Drivetrain Control --------------------------- */
        // Get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // Deadzone
		if (abs(leftY) < dead_zone) {
            leftY = 0;
        }
        if (abs(rightX) < dead_zone) {
            rightX = 0;
        }

        // Move motors
        left_drive.move(leftY + rightX);
        right_drive.move(leftY - rightX);

		/* ----------------------------- Intake Control ---------------------------- */
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake_speed = -120;

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake_speed = 120;

        } else {
            intake_speed = 0;
        }

        intake.move(intake_speed);

        /* ------------------------------ Arm Movement ------------------------------ */
        arm_angle = arm_rotation.get_position() / 100.0;

        // Up
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            arm_mode = 1;
        // Down
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            arm_mode = 2;
        // Bottom
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            arm_mode = 3;
        // Middle
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            arm_mode = 4;        
        // Neutral Stake
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            arm_mode = 5;
        // Goal Flip
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            arm_mode = 6;
        // Stop
        } else if (arm_mode < 3) {
            arm_mode = 0;
        }

        int modes[5] = {2, 56, 455, 695};
        
        if (arm_mode == 0) {
            arm.brake();
        } else if (arm_mode == 1) {
            arm.move(127);

        } else if (arm_mode == 2) {
            arm.move(-127);

        } else {
            arm_target = modes[arm_mode - 3];
            arm_difference = arm_target - arm_angle;
            arm_speed = arm_kP * arm_difference;

            // Check minimum speed
            if (arm_speed < min_arm_speed + arm_kD && arm_speed > 0) {
                arm_speed = min_arm_speed;

            } else if (arm_speed > -min_arm_speed - arm_kD && arm_speed < 0) {
                arm_speed = -min_arm_speed;
            }

            if (arm_difference <=  2 && arm_difference >= -2) {
                arm_speed = 0;
            }

            arm.move(arm_speed);
        }

		/* -------------------------------- Mogo Mech ------------------------------- */
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            clamp_down = !clamp_down;
			clamp.set_value(clamp_down);
        }

        /* --------------------------------- Sweeper -------------------------------- */
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            sweeper_down = !sweeper_down;
            sweeper.set_value(sweeper_down);
        }

		// Delay to save resources
		pros::delay(20);
    }
}
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
pros::adi::DigitalOut sweeper('A');
pros::adi::DigitalOut intake_lift('B');
pros::adi::DigitalOut clamp('C');

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
            pros::delay(85);
            intake.move(-60);
            pros::delay(200);
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

            pros::lcd::print(2, "X: %f", chassis.getPose().x);
            pros::lcd::print(3, "Y: %f", chassis.getPose().y);
            pros::lcd::print(4, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(5, "Arm Angle: %d", arm_angle);

            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // Setup
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    arm_rotation.set_position(57 * 100);

    // Alliance Stake
    armTo(605, 800, 0.6, 127, 8);
    //chassis.moveToPoint(0, -6, 1000);
    

    // Mobile Goal
    //chassis.turnToPoint(-28.5, -26.5, 800, {.forwards = false});
    chassis.moveToPoint(-28.5, -26.5, 1600, {
        .forwards = false,
        .maxSpeed = 70,
        .minSpeed = 15
    });

    chassis.waitUntil(5);
    armTo(2);
    chassis.waitUntilDone();
    clamp.set_value(true);

    // Line Rings
    chassis.turnToPoint(-32.5, -50, 800);
    pros::Task color_sort_task(colorSort);
    chassis.moveToPoint(-32.5, -50, 1000);

    //chassis.turnToPoint(-30, -58.5, 800);
    chassis.moveToPoint(-29, -59, 1000);

    // 3rd Ring
    chassis.moveToPoint(-33, -46.5, 800, {.forwards = false, .minSpeed = 40});
    
    chassis.turnToPoint(-24, -48.5, 800, {.minSpeed = 10});
    chassis.moveToPoint(-24, -48.5, 1000, {.minSpeed = 40});

    // Corner Ring
    chassis.turnToPoint(6.3, -37, 800, {.minSpeed = 10});
    chassis.moveToPoint(6.3, -37, 1000, {.minSpeed = 40});

    chassis.turnToPoint(25.5, -41, 800, {.minSpeed = 10});
    chassis.moveToPoint(25.5, -41, 800);

    // 5th Ring
    chassis.moveToPoint(7, -36, 800, {.forwards = false, .minSpeed = 40});

    chassis.turnToPoint(-11, -6.5, 800, {.minSpeed = 10});
    chassis.moveToPoint(-11, -6.5, 1000, {.minSpeed = 120, .earlyExitRange = 10});
    chassis.moveToPoint(-28.5, 23.5, 2000, {.maxSpeed = 20});

    // Ending
    clamp_down = true;

    // For Testing
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
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

    // Incase auto sort didn't derminate during auton
    if (colorSorter != nullptr) {
        delete colorSorter;
    }

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
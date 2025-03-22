/* -------------------------------- Includes -------------------------------- */
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include <cstdio>
#include "main.h"

/* ------------------------------- Controller ------------------------------- */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* --------------------------------- Motors --------------------------------- */
pros::MotorGroup left_drive({-3, -11, 12}, pros::MotorGearset::blue);
pros::MotorGroup right_drive({4, 2, -1}, pros::MotorGearset::blue);

pros::Motor intake(-13, pros::MotorGearset::blue);
pros::MotorGroup arm({-17, 5}, pros::MotorGearset::red);

/* --------------------------------- Sensors -------------------------------- */
pros::IMU inertial(14);
pros::Rotation arm_rotation(2);
pros::Rotation vert(6);
pros::Rotation hor(-19);

/* --------------------------------- Pistons -------------------------------- */
pros::adi::DigitalOut clamp('B');
pros::adi::DigitalOut sweeper('G');

/* ----------------------------- Tracking Wheels ---------------------------- */
lemlib::TrackingWheel vert_wheel(&vert, lemlib::Omniwheel::NEW_2, -1);
lemlib::TrackingWheel hor_wheel(&hor, lemlib::Omniwheel::NEW_2, 1);

/* ---------------------------- Drivetrain Setup ---------------------------- */
lemlib::Drivetrain drivetrain(
	&left_drive,
    &right_drive,
    10,
    lemlib::Omniwheel::NEW_275,
    450,
    2
);

lemlib::OdomSensors sensors(
	&vert_wheel,
    nullptr,
    &hor_wheel,
    nullptr,
    &inertial
);

/* ---------------------------------- PIDs ---------------------------------- */
lemlib::ControllerSettings lateral_controller(
    7,
    0,
    12,
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

void initialize() {
	/* ----------------------------- Motor Stopping ----------------------------- */
	left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    /* ---------------------------------- Setup --------------------------------- */
    pros::lcd::initialize();
    chassis.calibrate();
    arm_rotation.set_position(0);

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

void autonomous() {}

void opcontrol() {
	/* -------------------------------- Variables ------------------------------- */
	int dead_zone = 8;
	int intake_speed = 0;

	bool clamp_down = true;
    bool automatic_intake = false;
    bool sweeper_down = false;

    int arm_mode = 0;
    int arm_target;
    int arm_speed;

    float arm_angle = 0;
    float arm_kP = 1.5;
    float arm_difference;
    arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

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
            intake_speed = -127;

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake_speed = 127;

        } else {
            intake_speed = 0;
        }

        intake.move(intake_speed);

        /* ------------------------------ Arm Movement ------------------------------ */
        arm_angle = arm_rotation.get_position() / 100.0;
        if (arm_angle > 300) {
            arm_angle = 0;
        }
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
            arm_mode = 7;
        // Stop
        } else if (arm_mode < 3) {
            arm_mode = 0;
        }

        int modes[5] = {2, 25, 140, 210, 225};
        
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

            controller.clear();
            controller.print(1, 1, "Arm Speed:");

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
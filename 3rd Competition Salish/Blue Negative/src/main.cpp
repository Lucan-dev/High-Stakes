#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "main.h"

/* ------------------------------- Controller ------------------------------- */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* --------------------------------- Motors --------------------------------- */
pros::MotorGroup left_drive({-20, -16, 15}, pros::MotorGearset::blue);
pros::MotorGroup right_drive({-17, 18, 21}, pros::MotorGearset::blue);

pros::Motor intake(13, pros::MotorGearset::blue);
pros::MotorGroup arm({-4, 5}, pros::MotorGearset::red);

/* --------------------------------- Sensors -------------------------------- */
pros::IMU inertial(14);
pros::Rotation arm_rotation(2);
pros::Rotation vert(6);
pros::Rotation hor(-19);

/* --------------------------------- Pistons -------------------------------- */
pros::adi::DigitalOut clamp('H');
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

    arm.tare_position_all();
    intake.tare_position();

    arm.set_zero_position_all(0);
    intake.set_zero_position(0);
    arm_rotation.set_position(0);

    /* ---------------------------- Position on Brain --------------------------- */
    pros::Task screen_task([&]() {
        int angle;
        while (true) {
            angle = arm_rotation.get_position() / 100;
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::lcd::print(3, "Arm Angle: %d", angle);

            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {}

/* ------------------------------ My Functions ------------------------------ */
bool arm_up() {
    return (arm.get_position() > 2300 && arm.get_position() < 2500);
}

bool arm_down() {
    if (arm_rotation.get_angle() <= 50) {
        return false;

    } else {
        return true;
    }
}

void autonomous() {
    // Set drivetrain to hold
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    // Get goal
    chassis.moveToPoint(0, -10, 1000, {.forwards = false});
    chassis.turnToPoint(15.5, -27.5, 1000, {.forwards = false});
    chassis.moveToPoint(15.5, -27.5, 1500, {.forwards = false, .maxSpeed = 60});
    
    chassis.waitUntilDone();
    clamp.set_value(true);

    // 1st ring
    pros::delay(200);
    intake.move(127);
    pros::delay(500);

    chassis.turnToPoint(0, -42.5, 1000);
    chassis.moveToPoint(0, -42.5, 1500, {.maxSpeed = 80});

    // 2nd ring
    chassis.waitUntilDone();
    pros::delay(1000);

    chassis.turnToPoint(-5.5, -33, 1500, {.maxSpeed = 60});
    chassis.moveToPoint(-5.5, -33, 1000);

    // Touch bar
    chassis.waitUntilDone();
    pros::delay(1000);
    intake.move(-127);

    chassis.turnToPoint(20.5, -38.5, 1000);
    chassis.moveToPoint(20.5, -38.5, 2000);
}

void opcontrol() {
	/* -------------------------------- Variables ------------------------------- */
	int dead_zone = 8;
	int intake_speed = 0;
    int arm_speed = 0;
    int arm_angle = 0;

	bool clamp_down = false;
    bool automatic_intake = false;
    bool sweeper_down = false;

    // loop forever
    while (true) {
        arm_angle = arm_rotation.get_position() / 100;
        // pros::screen::erase();
        // pros::screen::print(pros::E_TEXT_LARGE, 3, "ArmPostion: %d", arm_angle);

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
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            arm_speed = 127;

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            arm_speed = -127;

        // Arm Middle
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            if (arm_angle > 30) {
                arm_speed = -30;

            } else if (arm_angle < 30) {
                arm_speed = 30;
            }

        } else if (arm_speed == -30) {
            if (arm_angle <= 32) {
                arm_speed = 0;
            }

        } else if (arm_speed == 30) {
            if (arm_angle >= 23) {
                arm_speed = 0;
            }

        // Arm Down
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            arm_speed = -129;

        } else if (arm_speed == -129) {
            if (arm_angle <= 15 || arm_angle >= 330) {
                arm_speed = 0;
            }

        // Arm Up
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            arm_speed = 129;

        } else if (arm_speed == 129) {
            if (arm_angle >= 145) {
                arm_speed = 0;
            }

        } else {
            arm_speed = 0;
        }

        if (arm_speed == 0) {
            arm.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
            arm.brake();

        } else {
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
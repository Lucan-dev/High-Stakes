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
pros::MotorGroup left_drive({-15, -14, -16}, pros::MotorGearset::blue);
pros::MotorGroup right_drive({17, 18, 19}, pros::MotorGearset::blue);

pros::Motor intake(-20, pros::MotorGearset::blue);

pros::Motor arm(-10, pros::MotorGearset::red);

/* --------------------------------- Sensors -------------------------------- */
pros::Imu inertial(13);
pros::Distance distance(9);

pros::Rotation rotation(8);
lemlib::TrackingWheel vert_wheel(&rotation, lemlib::Omniwheel::NEW_2, -1);

/* --------------------------------- Pistons -------------------------------- */
pros::adi::DigitalOut clamp('F');

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
    nullptr,
    nullptr,
    &inertial
);

/* ---------------------------------- PIDs ---------------------------------- */
lemlib::ControllerSettings lateral_controller(
    7,
    0,
    11,
    3,
    1,
    100,
    3,
    500,
    0
);

lemlib::ControllerSettings angular_controller(
    2,
    0,
    20,
    3,
    1,
    100,
    3,
    500,
    0
);

/* ----------------------------- Create Chassis ----------------------------- */
lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    sensors
);

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	/* ----------------------------- Motor Stopping ----------------------------- */
	left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);

    intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    /* ---------------------------------- Setup --------------------------------- */
    pros::lcd::initialize();
    chassis.calibrate();

    arm.tare_position();
    arm.set_zero_position(0);
    intake.tare_position();

    /* ---------------------------- Position on Brain --------------------------- */
    pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);

            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // Wall stake
    arm.move(127);
    while (arm.get_position() <= 1300) {
        pros::delay(50);
    }
    arm.brake();

    chassis.moveToPoint(0, 10, 800);
    chassis.waitUntilDone();

    arm.move(-127);
    while (arm.get_position() >= 1000) {
        pros::delay(50);
    }
    arm.brake();

    // Goal
    chassis.moveToPoint(0, 4, 600, {.forwards = false});

    chassis.turnToPoint(-9, -27, 600, {.forwards = false});

    arm.move(-127);
    while (arm.get_position() > 0) {
        pros::delay(50);
    }
    arm.brake();
    
    chassis.moveToPoint(-9, -27, 1200, {.forwards = false, .maxSpeed = 60});

    chassis.waitUntilDone();
    clamp.set_value(true);
    pros::delay(500);

    // Ring
    chassis.turnToPoint(6, -47, 700);
    intake.move(127);
    chassis.moveToPoint(6, -47, 1000, {.maxSpeed = 80});

    chassis.waitUntilDone();
    pros::delay(1500);
    clamp.set_value(false);
    intake.brake();

    // 2nd Goal
    pros::delay(500);
    chassis.moveToPoint(13.5, -57.6, 1200);

    chassis.turnToPoint(3.7, -61.4, 900, {.forwards = false});
    chassis.moveToPoint(3.7, -61.4, 1500, {.forwards = false, .maxSpeed = 60});

    chassis.waitUntilDone();
    clamp.set_value(true);

    // Touch bar
    pros::delay(600);
    chassis.turnToPoint(-25, -33, 900, {.forwards = false});
    chassis.moveToPoint(-25, -33, 1200, {.forwards = false});
}

/* ------------------------------ My Functions ------------------------------ */
bool arm_up() {
    return (arm.get_position() > 2300 && arm.get_position() < 2500);
}

bool arm_down() {
    return arm.get_position() <= 0;
}

void opcontrol() {
	/* -------------------------------- Variables ------------------------------- */
	int dead_zone = 8;
	int intake_speed = 0;
    int arm_speed = 0;
	bool clamp_down = false;
    bool automatic_intake = false;

    auto timeFlag=pros::millis();

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
        // Auto mode
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
            automatic_intake = !automatic_intake;
        }
        if (automatic_intake) {
            if (distance.get() <= 40) {
                intake.set_zero_position(0);
                intake_speed = -100;

            } else if (intake_speed == -100) {
                if (intake.get_position() <= -1500) {
                    intake_speed = 0;
                    automatic_intake = false;
                }

            } else {
                intake_speed = 127;
            }

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
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

        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            if (arm.get_position() < 2100) {
                arm_speed = 128;
            } else if (arm.get_position() > 2200) {
                arm_speed = -128;
            }

        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            arm_speed = -129;

        } else if (arm_speed == 128) {
            if (arm_up()) {
                arm_speed = 0;
            }

        } else if (arm_speed == -128) {
            if (arm_up()) {
                arm_speed = 0;
            }

        } else if (arm_speed == -129) {
            if (arm_down()) {
                arm_speed = 0;
            }

        } else {
            arm_speed = 0;
        }

        if (arm_speed == 0) {
            if (arm.get_position() > 500) {
                arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                arm.brake();
            } else {
                arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                arm.move(0);
            }
        } else {
            arm.move(arm_speed);
        }

		/* -------------------------------- Mogo Mech ------------------------------- */
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            clamp_down = !clamp_down;
			clamp.set_value(clamp_down);
        }

		// Delay to save resources
		pros::delay(20);
    }
}
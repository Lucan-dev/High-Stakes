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
	left_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    right_drive.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);

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
    while (arm.get_position() <= 1500) {
        pros::delay(50);
    }
    arm.brake();

    chassis.moveToPoint(0, 5, 700);
    chassis.waitUntilDone();

    arm.move(-127);
    while (arm.get_position() >= 900) {
        pros::delay(50);
    }
    arm.brake();

    // Goal
    chassis.moveToPoint(0, 1, 600, {.forwards = false});

    chassis.turnToPoint(-17, -4.5, 600, {.forwards = false});
    chassis.moveToPoint(-17, -4.5, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    clamp.set_value(true);

    // 6 Rings
    arm.move(-127);
    while (arm.get_position() > 10) {
        pros::delay(30);
    }
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    arm.move(0);

    chassis.turnToPoint(-21, -25, 700);
    chassis.moveToPoint(-21, -25, 1000);
    intake.move(127);

    chassis.turnToHeading(265, 700);
    chassis.moveToPose(-57, -58, 201, 2000, {.minSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(300);

    chassis.turnToHeading(375, 800, {.maxSpeed = 70});
    chassis.moveToPose(-48, 2, -3, 2200);
    pros::delay(500);

    chassis.turnToHeading(-115, 600);
    chassis.moveToPoint(-59, -9, 600);

    // Place goal in corner
    chassis.turnToHeading(-208, 900);
    chassis.moveToPoint(-59, 0, 500, {.forwards = false});
    chassis.waitUntilDone();
    clamp.set_value(false);

    // 2nd goal
    intake.move(-127);
    chassis.moveToPoint(-60, -6, 1000);
    chassis.turnToPoint(14, -4, 1000, {.forwards = false});
    chassis.moveToPoint(14, -4, 2000, {.forwards = false});

    chassis.turnToHeading(-80, 800, {.minSpeed = 70});
    chassis.turnToHeading(-90, 800, {.minSpeed = 70});
    chassis.moveToPoint(21, -4, 1000, {.maxSpeed = 60});

    chassis.waitUntilDone();
    clamp.set_value(true);

    // 6 rings
    pros::delay(300);
    chassis.turnToPoint(27, -23, 700);
    chassis.moveToPoint(27, -23, 1000);
    intake.move(127);

    chassis.turnToHeading(85, 700);
    chassis.moveToPose(65, -58, 159, 2000, {.minSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(300);

    chassis.turnToHeading(-17, 800, {.maxSpeed = 70});
    chassis.moveToPose(52, 9, 1, 2200);
    pros::delay(500);

    chassis.turnToPoint(65, -2.5, 700);
    chassis.moveToPoint(65, -2.5, 700);

    // Place goal in corner
    chassis.turnToHeading(193, 900);
    chassis.moveToPoint(67, 4, 700);
    chassis.waitUntilDone();
    clamp.set_value(false);
    intake.move(-127);

    // Get wall stake ring
    chassis.turnToPoint(57, -76, 700);
    chassis.moveToPose(57, -76, 189, 2500);
    intake.move(127);

    while (distance.get() >= 40) {
        intake.move(127);
        pros::delay(30);
    }
    intake.brake();
    intake.set_zero_position(0);

    while (intake.get_position() >= -1500) {
        intake.move(-127);
        pros::delay(20);
    }
    intake.brake();

    // Wall stake
    chassis.moveToPoint(60, -51, 1000, {.forwards = false, .maxSpeed = 60});
    while (arm.get_position() <= 2300) {
        arm.move(127);
        pros::delay(20);
    }
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    arm.brake();

    chassis.turnToHeading(90, 700, {.maxSpeed = 60});
    chassis.moveToPoint(72.5, -51, 1000, {.maxSpeed = 60});
    chassis.waitUntilDone();

    while (arm.get_position() > 1600) {
        arm.move(-127);
        pros::delay(20);
    }
    arm.brake();

    // Ring
    chassis.moveToPoint(63, -52, 800, {.forwards = false});

    chassis.turnToPoint(31, -79, 800);
    intake.move(127);
    chassis.moveToPoint(31, -79, 2000, {.maxSpeed = 100});

    arm.move(-127);
    while (arm.get_position() > 10) {
        pros::delay(30);
    }
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    arm.move(0);

    chassis.waitUntilDone();
    pros::delay(200);
    intake.brake();

    // Goal
    chassis.turnToPoint(24, -105.5, 800);
    chassis.moveToPoint(24, -105.5, 1000);

    chassis.turnToHeading(129.5, 1000);
    chassis.moveToPose(66, -115.5, 88.5, 2000, {.minSpeed = 90});

    // // 3 rings
    // chassis.moveToPose(37, -106, 478, 1500, {.forwards = false, .minSpeed = 80});
    // intake.move(127);
    // chassis.turnToPoint(59, -99, 800);
    // chassis.moveToPoint(59, -99, 1000);
    
    // chassis.turnToHeading(250, 900);
    // chassis.moveToPose(-16, -83, 676, 3000, {.minSpeed = 60});

    // chassis.waitUntilDone();
    // pros::delay(500);

    // // Place goal in corner
    // chassis.moveToPoint(-11, -92, 800, {.forwards = false});

    // chassis.turnToPoint(-49, -98, 800, {.forwards = false});
    // chassis.moveToPoint(-49, -98, 1000, {.forwards = false});
    // intake.brake();

    // chassis.turnToHeading(755, 800);
    // pros::delay(100);
    // clamp.set_value(false);
    // chassis.moveToPose(-55, -135, 0, 1500, {.forwards = false, .minSpeed = 70});

    // // Get 2nd wall stake ring
    // chassis.turnToPoint(-42, -64.5, 800);
    // chassis.moveToPoint(-42, -64.5, 3000, {.maxSpeed = 60});
    // intake.move(127);

    // while (distance.get() >= 40) {
    //     intake.move(127);
    //     pros::delay(30);
    // }
    // intake.brake();
    // intake.set_zero_position(0);

    // while (intake.get_position() >= -1500) {
    //     intake.move(-127);
    //     pros::delay(20);
    // }
    // intake.brake();

    // chassis.waitUntilDone();

    // // 2nd wall stake
    // chassis.turnToPoint(-66, -64.5, 1000, {.maxSpeed = 60});

    // while (arm.get_position() <= 2300) {
    //     arm.move(127);
    //     pros::delay(20);
    // }
    // arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // arm.brake();

    // chassis.moveToPoint(-66.5, -64.5, 2000, {.maxSpeed = 80});
    // chassis.waitUntilDone();

    // while (arm.get_position() > 1700) {
    //     arm.move(-127);
    //     pros::delay(20);
    // }
    // arm.brake();

    // // Leave
    // pros::delay(200);
    // chassis.moveToPoint(-55, -66, 2000, {.forwards = false});

    // Coast
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
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

    // Wall stake
    arm.move(127);
    while (arm.get_position() <= 1400) {
        pros::delay(50);
    }
    arm.brake();

    chassis.moveToPoint(0, 5, 700);
    chassis.waitUntilDone();

    arm.move(-127);
    while (arm.get_position() >= 900) {
        pros::delay(50);
    }
    arm.brake();

    // Goal
    chassis.moveToPoint(0, 1, 600, {.forwards = false});

    chassis.turnToPoint(-17, -4.5, 600, {.forwards = false});
    chassis.moveToPoint(-17, -4.5, 1000, {.forwards = false, .maxSpeed = 60});

    arm.move(-127);
    while (arm.get_position() > 10) {
        pros::delay(30);
    }
    arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    arm.move(0);

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
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
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

void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    int intake_speed = 120;
    int arm_middle_angle = 24;
    
    // Score red wall stake
    intake.move(127);
    pros::delay(600);
    intake.move(-127);

    // Get 1st mobile goal
    chassis.moveToPoint(0, 16, 1000);
    chassis.turnToHeading(-90, 1000);
    chassis.moveToPoint(19, 15, 1000, {.forwards = false, .maxSpeed = 60});

    chassis.waitUntilDone();
    clamp.set_value(true);
    pros::delay(200);

    // 2 rings
    chassis.turnToHeading(0, 1000);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPose(28, 46.5, 23, 1000, {.minSpeed = 80});

    intake.move(intake_speed);
    chassis.moveToPose(48.5, 87, 8, 2000, {.minSpeed = 80});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    chassis.waitUntilDone();

    // Wallstake ring
    float wallstake_y = 62;
    chassis.turnToPoint(43, wallstake_y + 1, 1000);
    chassis.moveToPoint(43, wallstake_y + 1, 1500, {.maxSpeed = 100});

    chassis.turnToPoint(63, wallstake_y, 1000, {.maxSpeed = 80});
    while (arm_rotation.get_position() / 100 <= arm_middle_angle) {
        arm.move(50);
        pros::delay(30);
    }
    arm.brake();
    chassis.moveToPoint(53, wallstake_y, 1000);
    chassis.waitUntilDone();
    pros::delay(600);
    chassis.moveToPoint(62, wallstake_y, 2000, {.maxSpeed = 30});

    // Score 1st neutral wallstake
    chassis.waitUntilDone();
    intake.brake();

    while (arm_rotation.get_position() / 100 <= 150) {
        arm.move(127);
        pros::delay(30);
    }
    arm.brake();
    pros::delay(200);
    while (arm_rotation.get_position() / 100 >= 10) {
        arm.move(-127);
        pros::delay(30);
    }
    arm.brake();

    // 4 more goal rings
    chassis.moveToPoint(45, 60, 1000, {.forwards = false});
    chassis.turnToPoint(43, 0, 1000);
    chassis.moveToPoint(43, 0, 3000, {.maxSpeed = 80});
    intake.move(intake_speed);

    chassis.turnToPoint(58, 10, 1000);
    chassis.moveToPoint(58, 10, 1000);

    // Place 1st goal in corner
    chassis.turnToPoint(59, 2, 1000, {.forwards = false});
    chassis.moveToPoint(59, 2, 1000, {.forwards = false});
    chassis.waitUntilDone();
    intake.move(-127);
    clamp.set_value(false);

    // Get 2nd goal
    chassis.moveToPoint(55, 13, 1000);

    float goal_y = 15;
    chassis.turnToPoint(-18, goal_y, 1000, {.forwards = false});
    chassis.moveToPoint(-18, goal_y, 2000, {.forwards = false}, false);

    chassis.moveToPoint(-26, goal_y, 1000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    clamp.set_value(true);
    pros::delay(200);

    // Get 1 ring
    chassis.turnToPoint(-26.5, 33.5, 1000);
    intake.move(intake_speed);
    chassis.moveToPoint(-26.5, 33.5, 1000);

    // Get wallstake ring
    chassis.turnToPoint(-62, 69, 1000);
    chassis.moveToPoint(-62, 69, 1000);
    chassis.waitUntil(30);
    while (arm_rotation.get_position() / 100 <= arm_middle_angle) {
        arm.move(50);
        pros::delay(30);
    }
    arm.brake();

    chassis.waitUntilDone();
    pros::delay(200);

    // Score 2nd neutral wallstake
    chassis.turnToPoint(-67, 68, 1000);
    chassis.moveToPoint(-67, 68, 1000, {.maxSpeed = 80});
    chassis.waitUntilDone();

    intake.brake();
    while (arm_rotation.get_position() / 100 <= 150) {
        arm.move(127);
        pros::delay(30);
    }
    arm.brake();

    // Get 4 rings
    intake.move(-127);
    while (arm_rotation.get_position() / 100 >= 10) {
        arm.move(-127);
        pros::delay(30);
    }
    arm.brake();
    chassis.moveToPoint(-51, 68, 1000, {.forwards = false});

    chassis.turnToPoint(-54, 7, 1000);
    chassis.moveToPoint(-54, 7, 2000, {.maxSpeed = 80});
    intake.move(intake_speed);

    chassis.turnToPoint(-64, 15, 1000);
    chassis.moveToPoint(-64, 15, 1000);

    // Place 2nd mogo in corner
    chassis.turnToPoint(-70, 7, 1000, {.forwards = false});
    chassis.moveToPoint(-70, 7, 1000, {.forwards = false});
    chassis.waitUntilDone();
    intake.move(-127);
    clamp.set_value(false);

    // Get ring
    chassis.turnToPoint(-52, 88, 1000);
    chassis.moveToPoint(-52, 88, 2000);
    intake.move(intake_speed);

    // Get 3rd mogo
    chassis.turnToPoint(-33, 117, 1000);
    pros::delay(400);
    intake.brake();
    chassis.moveToPoint(-33, 117, 1000);

    chassis.waitUntilDone();
    sweeper.set_value( true);
    pros::delay(200);

    // Place 3rd mogo in corner
    chassis.swingToHeading(-415, lemlib::DriveSide::LEFT, 1500);
    chassis.moveToPose(-55, 135, -435, 1500, {.minSpeed = 100});

    // Coast motors
    chassis.waitUntilDone();
    pros::delay(200);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
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

    // Score red wall stake
    intake.move(127);
    pros::delay(600);
    intake.move(-127);

    // Get 1st mobile goal
    chassis.moveToPoint(0, 16, 1000);
    chassis.turnToHeading(-90, 1000);
    chassis.moveToPoint(19, 15, 1000, {.forwards = false, .maxSpeed = 60});

    chassis.waitUntilDone();
    clamp.set_value(true);
    pros::delay(200);

    clamp_down = true;

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
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/link.h"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include <cstdio>
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
lemlib::TrackingWheel hor_wheel(&hor, lemlib::Omniwheel::NEW_2, 0.75);

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
            angle = arm_rotation.get_position() / -100;
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

void print_coords() {
    double rounded_x;
    double rounded_y;
    double heading;
    int arm_deg;

    while (true) {
        rounded_x = std::round(chassis.getPose().x * 100.0) / 100.0;
        rounded_y = std::round(chassis.getPose().y * 100.0) / 100.0;
        heading = std::round(chassis.getPose().theta * 100.0) / 100.0;
        arm_deg = arm_rotation.get_position() / -100;

        std::cout << "x: " << rounded_x << ", y: " << rounded_y << std::endl;
        std::cout << "Heading:" << heading << ", Arm angle: " << arm_deg << std::endl;
        std::cout << "" << std::endl;

        pros::delay(500);
    }
}

int arm_down = 2;
int arm_middle = 22;
int arm_up = 140;
int arm_flip = 225;
int arm_overshoot = 6;

void arm_to_down() {
    while (arm_rotation.get_position() / 100 >= arm_down + arm_overshoot) {
        arm.move(-127);
    }
    arm.brake();
}

void arm_to_middle() {
    while (arm_rotation.get_position() / 100 <= arm_middle - arm_overshoot) {
        arm.move(30);
    }
    arm.brake();
}

void arm_to_up() {
    while (arm_rotation.get_position() / 100 <= arm_up - arm_overshoot) {
        arm.move(127);
    }
    arm.brake();
}

void autonomous() {
    // Setup
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    /* --------------------------------- Motion --------------------------------- */
    
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

    // 1st ring
    chassis.turnToPoint(25, 41, 1000);
    chassis.moveToPoint(25, 41, 2000);
    intake.move(127);

    // Wallstake Ring
    chassis.turnToPoint(59.5, 65.5, 1000);
    chassis.moveToPoint(59.5, 65.5, 2000, {.maxSpeed = 100});
    intake.move(80);

    arm_to_middle();

    // Score Wallstake
    chassis.turnToHeading(90, 1000, {.maxSpeed = 60});
    chassis.moveToPoint(68, 66.5, 700);

    chassis.waitUntilDone();
    intake.brake();
    pros::delay(100);

    arm_to_up();

    // Get 2nd Ring
    chassis.moveToPoint(57, 64.5, 1000, {.forwards = false});

    chassis.turnToPoint(52, 85, 1000);
    intake.move(127);
    arm_to_down();
    chassis.moveToPoint(52, 85, 1000);

    // Get 3 more rings
    chassis.turnToPoint(45, 5, 1000);
    chassis.moveToPoint(45, 5, 2500, {.maxSpeed = 80});

    chassis.turnToPoint(58, 15, 1000);
    chassis.moveToPoint(58, 15, 1000);

    // Place goal in corner
    chassis.turnToPoint(60, 8, 1000, {.forwards = false});
    chassis.moveToPoint(60, 8, 1000, {.forwards = false, .minSpeed = 20});

    chassis.waitUntilDone();
    intake.move(-127);
    clamp.set_value(false);

    // 2nd Goal
    chassis.moveToPoint(57, 12, 1000, {.minSpeed = 20});

    chassis.turnToPoint(15.5, 21, 1000, {.forwards = false});
    chassis.moveToPoint(-15.5, 21, 2000, {.forwards = false});
    intake.brake();

    chassis.moveToPoint(-24, 21, 800, {.forwards = false, .maxSpeed = 60});
    
    chassis.waitUntilDone();
    clamp.set_value(true);

    // 1st Ring
    pros::delay(200);
    chassis.turnToPoint(-28, 47, 1000);
    chassis.moveToPoint(-28, 47, 1000);
    intake.move(127);

    // 2nd Wallstake Ring
    chassis.turnToPoint(-58, 73.5, 1000);
    chassis.moveToPoint(-58, 73.5, 2000, {.maxSpeed = 100});
    intake.move(80);

    arm_to_middle();

    // Score 2nd Wallstake
    chassis.turnToHeading(-90, 1000, {.maxSpeed = 60});
    chassis.moveToPoint(-66, 75, 700);

    chassis.waitUntilDone();
    intake.brake();
    pros::delay(100);

    arm_to_up();

    // Get 2nd Ring
    chassis.moveToPoint(-56.5, 70.5, 1000, {.forwards = false});

    chassis.turnToPoint(-49, 93, 1000);
    intake.move(127);
    arm_to_down();

    chassis.moveToPoint(-49, 93, 1000);

    // 3 More Rings
    chassis.turnToPoint(-49, 13, 1000);
    chassis.moveToPoint(-49, 13, 2500, {.maxSpeed = 80});

    // Last Ring
    chassis.turnToPoint(-62, 25, 1000);
    chassis.moveToPoint(-62, 25, 1000);

    // Place 2nd Goal in Corner
    chassis.turnToPoint(-64, 18, 1000, {.forwards = false});
    chassis.moveToPoint(-64, 18, 1000, {.forwards = false});

    chassis.waitUntilDone();
    intake.move(-127);
    clamp.set_value(false);

    // 1st Ring
    chassis.moveToPose(-27, 97, 35, 3000, {.minSpeed = 100});

    chassis.waitUntil(20);
    intake.move(127);

    // Push 3rd goal into corner
    chassis.turnToPoint(-21, 128, 1000);
    pros::delay(100);
    intake.brake();
    chassis.moveToPoint(-21, 128, 1000);

    chassis.swingToHeading(-80, lemlib::DriveSide::LEFT, 1200);
    chassis.moveToPoint(-52, 140, 1200);

    // Get 4th goal
    chassis.moveToPoint(0, 124, 4000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    clamp.set_value(true);

    // 1 ring
    pros::delay(200);
    chassis.turnToPoint(26, 96, 1000);
    chassis.moveToPoint(26, 96, 1000);
    intake.move(127);

    // 2nd ring
    chassis.turnToPoint(46, 114, 1000);
    chassis.moveToPoint(46, 114, 1000);

    // 3rd ring
    chassis.turnToPoint(49, 126, 1000);
    chassis.moveToPoint(49, 126, 1000);

    // Place goal in corner
    chassis.waitUntilDone();
    pros::delay(300);
    sweeper.set_value(true);

    chassis.turnToPoint(62, 132, 1000, {.forwards = false, .direction = lemlib::AngularDirection::CW_CLOCKWISE, .maxSpeed = 80});

    chassis.waitUntilDone();
    intake.brake();

    pros::delay(200);
    chassis.moveToPoint(62, 132, 1000, {.forwards = false});

    // Leave
    chassis.moveToPoint(30, 100, 50000);
    clamp.set_value(false);

    // Hang
    chassis.waitUntilDone();
    chassis.turnToHeading(45, 1000);
    arm_to_up();
    chassis.moveToPoint(5, 70, 1000, {.forwards = false, .maxSpeed = 100});
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    /* --------------------------------- Ending --------------------------------- */
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

    int arm_down = 2;
    int arm_middle = 25;
    int arm_up = 140;
    int arm_flip = 225;
    int arm_overshoot = 6;

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
        // Make it so when arm goes up the angle also goes up

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
            if (arm_angle < arm_middle) {
                arm_speed = 30;

            } else if (arm_angle > arm_middle) {
                arm_speed = -30;
            }

        } else if (arm_speed == 30) {
            // Go up
            if (arm_angle >= arm_middle - arm_overshoot) {
                arm_speed = 0;
            }

        } else if (arm_speed == -30) {
            // Go down
            if (arm_angle <= arm_middle + arm_overshoot) {
                arm_speed = 0;
            }

        // Arm Down
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            arm_speed = -128;

        } else if (arm_speed == -128) {
            if (arm_angle <= arm_down + arm_overshoot * 3) {
                arm_speed = 0;
            }

        // Arm Up
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            if (arm_angle < arm_up) {
                arm_speed = 129;
            } else {
                arm_speed = -129;
            }

        } else if (arm_speed == 129) {
            // Up
            if (arm_angle >= arm_up - arm_overshoot * 3) {
                arm_speed = 0;
            }

        } else if (arm_speed == -129) {
            // Down
            if (arm_angle <= arm_up + arm_overshoot * 3) {
                arm_speed = 0;
            }

        // Arm flip
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            if (arm_angle <= arm_flip) {
                arm_speed = 130;

            } else if (arm_angle >= arm_flip) {
                arm_speed = -130;
            }

        } else if (arm_speed == 130) {
            // Up
            if (arm_angle >= arm_flip - arm_overshoot * 3) {
                arm_speed = 0;
            }

        } else if (arm_speed == -130) {
            // Down
            if (arm_angle <= arm_flip + arm_overshoot * 3) {
                arm_speed = 0;
            }

        // Stop arm
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
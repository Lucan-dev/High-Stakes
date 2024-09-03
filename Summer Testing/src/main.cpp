#include "main.h"
#include "lemlib/api.hpp"

/* ------------------------------- Controller ------------------------------- */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/* ---------------------------- Drivetrain Motors --------------------------- */
pros::Motor left_front(-9, pros::E_MOTOR_GEAR_BLUE);     // port 1, reversed, blue cart
pros::Motor left_back(-19, pros::E_MOTOR_GEAR_BLUE);     // port 2, reversed, blue cart

pros::Motor right_front(5, pros::E_MOTOR_GEAR_BLUE);   // port 3, forwards, blue cart
pros::Motor right_back(15, pros::E_MOTOR_GEAR_BLUE);   // port 4, forwards, blue cart

/* ------------------------------ Motor Groups ------------------------------ */
pros::MotorGroup left_drive({left_front, left_back});
pros::MotorGroup right_drive({right_front, right_back});

/* --------------------------------- Sensors -------------------------------- */
pros::Imu inertial(20);

pros::Rotation vert_rotation(11, true);    // forwards
pros::Rotation hor_rotation(21, true);     // forwards

/* -------------------------- Tracking Wheel Setup -------------------------- */
lemlib::TrackingWheel vert_tracking_wheel(&vert_rotation, lemlib::Omniwheel::NEW_275, -1);
lemlib::TrackingWheel hor_tracking_wheel(&hor_rotation, lemlib::Omniwheel::NEW_275, -2);


/* --------------------------- Drivetrain Setup -------------------------- */
lemlib::Drivetrain drivetrain(
	&left_drive, 				// left motor group
    &right_drive, 				// right motor group
    10, 						// 10" track width
    lemlib::Omniwheel::NEW_275,	// using new 2.75" omnis
	450, 						// drivetrain rpm is 450
    2 							// horizontal drift is 2
);

lemlib::OdomSensors sensors(
	&vert_tracking_wheel, 		// Vert tracking 1
    nullptr, 					// Vert trackin 2
    &hor_tracking_wheel, 		// Hor tracking 1
    nullptr, 					// Hor tracking 2
    &inertial 					// Inertial
);

/* -------------------------------- PID Setup ------------------------------- */
lemlib::ControllerSettings lateral_controller(
	8,		// Proportional gain (kP)
    0,		// Integral gain (kI)
    60,		// Derivative gain (kD)
    0,		// Anti windup
    1,		// Small error range, in inches
    100,	// Small error range timeout, in milliseconds
    3, 		// Large error range, in inches
	500, 	// Large error range timeout, in milliseconds
    4 		// Maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(
	2, 		// Proportional gain (kP)
    0, 		// Integral gain (kI)
    17, 	// Derivative gain (kD)
    0, 		// Anti windup
    1, 		// Small error range, in degrees
    100, 	// Small error range timeout, in milliseconds
    3, 		// Large error range, in degrees
    500, 	// Large error range timeout, in milliseconds
    0 		// Maximum acceleration (slew)
);

/* ----------------------------- Create Chassis ----------------------------- */
lemlib::Chassis chassis(
    drivetrain,         // Drivetrain settings
    lateral_controller, // Lateral PID settings
    angular_controller, // Angular PID settings
    sensors             // Odometry sensors
);


/* -------------------------------- Variables ------------------------------- */
int dead_zone = 5;

/* -------------------------------------------------------------------------- */
/*                              Control Functions                             */
/* -------------------------------------------------------------------------- */

void initialize() {
	pros::lcd::initialize();
    chassis.calibrate();

    /* ----------------------------- Motor Stopping ----------------------------- */
    left_drive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
    right_drive.set_brake_modes(pros::E_MOTOR_BRAKE_HOLD);
	
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); 			// X position
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); 			// Y position
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);	// Heading degrees

            // delay to save resources
            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    chassis.turnToHeading(180, 2000);
}

void opcontrol() {
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

		// Delay to save resources
		pros::delay(20);
	}
}
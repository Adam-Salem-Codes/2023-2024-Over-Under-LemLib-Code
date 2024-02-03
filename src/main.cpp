// Including necessary libraries/APIs
#include "main.h"
#include "selection.h"
// Including necessary libraries/APIs

/*

  __ ___  ____  _____  _____   ___  _____    _____                             _____          _         
 /_ |__ \|___ \| ____|/ _ \ \ / ( )/ ____|  / ____|                           / ____|        | |        
  | |  ) | __) | |__ | | | \ V /|/| (___   | (___   ___  _   _ _ __ ___ ___  | |     ___   __| | ___    
  | | / / |__ <|___ \| | | |> <    \___ \   \___ \ / _ \| | | | '__/ __/ _ \ | |    / _ \ / _` |/ _ \   
  | |/ /_ ___) |___) | |_| / . \   ____) |  ____) | (_) | |_| | | | (_|  __/ | |___| (_) | (_| |  __/   
  |_|____|____/|____/ \___/_/_\_\_|_____/ _|_____/ \___/ \__,_|_|  \___\___|  \_____\___/ \__,_|\___|   
 |__ \ / _ \__ \|___ \    |__ \ / _ \__ \| || |    / __ \                 | |  | |         | |          
    ) | | | | ) | __) |_____ ) | | | | ) | || |_  | |  | |_   _____ _ __  | |  | |_ __   __| | ___ _ __ 
   / /| | | |/ / |__ <______/ /| | | |/ /|__   _| | |  | \ \ / / _ \ '__| | |  | | '_ \ / _` |/ _ \ '__|
  / /_| |_| / /_ ___) |    / /_| |_| / /_   | |   | |__| |\ V /  __/ |    | |__| | | | | (_| |  __/ |   
 |____|\___/____|____/  _ |____|\___/____|  |_|    \____/  \_/_\___|_|     \____/|_| |_|\__,_|\___|_|   
  ________    _       __      ___                          ____
 /_  __/ /_  (_)___  / /__   /   |_      ______ __________/ / /
  / / / __ \/ / __ \/ //_/  / /| | | /| / / __ `/ ___/ __  / / 
 / / / / / / / / / / ,<    / ___ | |/ |/ / /_/ / /  / /_/ /_/  
/_/ /_/ /_/_/_/ /_/_/|_|  /_/  |_|__/|__/\__,_/_/   \__,_(_)                                    
*/

// Using the namespaces to avoid having to type "pros::"
using namespace pros;
using namespace lemlib;
using namespace selector;
// Using the namespaces to avoid having to type "pros::"

Motor intake(5, E_MOTOR_GEARSET_06, false); // Backward to intake
// Catapult motors.
Motor cata1(6, E_MOTOR_GEARSET_36, false);	// Forward to pull back
Motor cata2(7, E_MOTOR_GEARSET_36, false);
MotorGroup cata({cata1, cata2});
// Catapult motors.

// Pneumatics
ADIDigitalOut wings('G');
ADIDigitalOut intake_cylinder('H');
// Pneumatics

// Drivetrain init
Motor left_front_motor(1, E_MOTOR_GEARSET_06, true); // Reversed
Motor left_back_motor(2, E_MOTOR_GEARSET_06, true); // Reversed
Motor right_front_motor(3, E_MOTOR_GEARSET_06, false); // Normal
Motor right_back_motor(4, E_MOTOR_GEARSET_06, false); // Normal
MotorGroup left_side_motors({left_front_motor, left_back_motor});
MotorGroup right_side_motors({right_front_motor, right_back_motor});
// LemLib drivetrain setup for use with Odometry and Pure Pursuit
Drivetrain_t drivetrain{
	&left_side_motors,	// left Drivetrain motors
	&right_side_motors, // Right Drivetrain motors
	14,				// How long the sides of the Drivetrain are.
	lemlib::Omniwheel::OLD_4,	// Wheel Diameter. Old Omnis are 3.18 inches in diameter.
	257					// Wheel RPM.
};
// LemLib drivetrain setup for use with Odometry and Pure Pursuit

pros::Imu inertial_sensor(17); // Port 17

// Settings for Odometry sensors
OdomSensors_t sensors{
	nullptr,			// We don't have tracking wheels so nullptr. (vert wheel 1)
	nullptr,			// Vertical tracking wheel 2
	nullptr,			// Horizontal tracking wheel 1
	nullptr,			// Horizontal tracking wheel 2
	&inertial_sensor	// Inertial sensor (IMU)
};
// Settings for Odometry sensors

// PID for driving straight (Lateral PID Controller)
lemlib::ChassisController_t lateralController{
    15, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    60 // slew rate
};
// PID for driving straight (Lateral PID Controller)

// PID for turning (Angular PID Controller)
ChassisController_t angularController{
	5,	// kP
	8, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500 // largeErrorTimeout
};
// PID for turning (Angular PID Controller)

Chassis chassis(drivetrain, lateralController, angularController, sensors);
// Drivetrain init

Controller master(pros::E_CONTROLLER_MASTER);

// All autonomous paths
ASSET(skills_txt);
ASSET(red_left_txt);
ASSET(movement_txt);
ASSET(path_txt)
ASSET(skills2_txt);
ASSET(red_right_txt);
// All autonomous paths

// Updates the controller with the current x, y, and orientation of the robot.
void update_controller() {
	while(true) { // Runs forever inside of a task (runs in the background)
		// This is mostly for debugging or noticing if something goes wrong.
		master.print(0, 0, std::to_string(round(chassis.getPose().x)).c_str());
		master.print(1, 0, std::to_string(round(chassis.getPose().y)).c_str());
		master.print(2, 0, std::to_string(round(chassis.getPose().theta)).c_str());
		/* 
		std::to_string converts the pose of the robot to a string.
		I rounded it to avoid unnecessary information like an overflow of zeros
		c_str converts the C++ string into a C style string (char pointer)
		*/
		delay(100); // Delay (100ms) to prevent updating the controller too much taking up resources.
	}
}
// Updates the controller with the current x, y, and orientation of the robot.

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	//pros::lcd::initialize();
	chassis.calibrate(); // Calibrates the Chassis (Calibrates the IMU and other sensors).
	Task screen_task(update_controller); // Updates the controller forever.
	selector::init(130, 1, selector::b); // Initializes the Autonomous selector.
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous()
{
	chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0 It zeros the robots position and orientation.
	intake_cylinder.set_value(true); // All of our autonomous programs require that the intake be out.
	switch (selector::auton)
	{
		case 0:
			// SKILLS AUTON
			//	wings.set_value(true);
			// Sets the starting position.
			chassis.setPose(-33, -60, 0);
			// Sets the starting position.

			// Follows the skills path.
			chassis.follow(path_txt, 100000, 7, true, true, 127, false);
			// Follows the skills path.
			
			// Runs specifically at distance 74 asynchronously.
			chassis.waitUntilDist(74);
			// Shoots catapult for match-loads.
			cata.move(127);
			pros::delay(30000);
			cata.move(0);
			
			chassis.waitUntilDist(139);
			// Sets motors to 100% Forward to get over barrier.
			left_side_motors.move(127);
			right_side_motors.move(127);
			pros::delay(1000);
			wings.set_value(true); // Pushes Tri-balls in front of goal.
			pros::delay(2000);
			left_side_motors.move(0); // Stops the motors.
			right_side_motors.move(0);
			chassis.turnTo(36, 8, 500, false, false, 127, false);
			chassis.setPose(45, 0, 0); // Estimates where the robot will be after barrier hop.
			chassis.follow(skills2_txt, 100000, 7, true, true, 127, false); // Does a side push into goal after main push.
			break;
 	    case 1:
			// RED LEFT AUTON
			chassis.setPose(-35, -59, 0);
			chassis.follow(red_left_txt, 100000, 7, true, true, 127, false);
			chassis.waitUntilDist(100);
			intake.move(-127);
			chassis.waitUntilDist(130);
			wings.set_value(true);
			left_side_motors.move(127);
			right_side_motors.move(127);
			break;
		case 2:
			// RED RIGHT AUTON

			break;
		case 3:
			// DO NOTHING
			break;
		case -1:
			// BLUE LEFT AUTON
			break;
		case -2:
			// BLUE RIGHT AUTON
			break;
		case -3:
		 	// DO NOTHING AGAIN...
		 	break;
		default:
			break;
	}

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
	while (true)
	{
		int forward = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y); // Gets the joystick value.
		int turn = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X); // Gets the joystick value.
		// Calculating the left and right motor velocities.
		left_side_motors.move(forward + turn);
		right_side_motors.move(forward - turn);
		// Calculating the left and right motor velocities.
		if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) 
			intake.move(127); // Moves intake forward when you press the R1 Controller button.
		else if (master.get_digital(E_CONTROLLER_DIGITAL_R2))
			intake.move(-127); // Moves intake backward when you press the R2 Controller button.
		else
			intake.move(0); // Stops when you aren't pressing anything.


		if (master.get_digital(E_CONTROLLER_DIGITAL_L1))
			cata.move(127); // Moves catapult forward when you press the L1 Controller button.
		else if (master.get_digital(E_CONTROLLER_DIGITAL_L2))
			cata.move(-127); // Moves catapult backward when you press the L2 Controller button.
		else
			cata.move(0); // Stops when you aren't pressing anything.
		pros::delay(10); // Delay of 10ms to not take up too many resources but allow for fast refresh for better control.
	}
}

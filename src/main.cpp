#include "main.h"
#include "lemlib/api.hpp"
#include "selection.h"
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
using namespace pros;
using namespace lemlib;
using namespace selector;

Motor left_front_motor(1, E_MOTOR_GEARSET_06, true);
Motor left_back_motor(2, E_MOTOR_GEARSET_06, true);
Motor right_front_motor(3, E_MOTOR_GEARSET_06, false);
Motor right_back_motor(4, E_MOTOR_GEARSET_06, false);
Motor intake(5, E_MOTOR_GEARSET_06, false); // backward to intake
Motor cata(6, E_MOTOR_GEARSET_36, false);	// Forward to pull back
ADIDigitalOut left_wing('B');
ADIDigitalOut right_wing('C');
MotorGroup left_side_motors({left_front_motor, left_back_motor});
MotorGroup right_side_motors({right_front_motor, right_back_motor});

Drivetrain_t drivetrain{
	&left_side_motors,	// left drivetrain motors
	&right_side_motors, // right drivetrain motors
	14,				// track width
	lemlib::Omniwheel::OLD_4,	// wheel diameter
	257					// wheel rpm
};
pros::Imu inertial_sensor(17); // port 7

// odometry struct
OdomSensors_t sensors{
	&first_tracking,			  // vertical tracking wheel 1
	nullptr,			  // vertical tracking wheel 2
	nullptr,//&back_tracking_wheel, // horizontal tracking wheel 1
	nullptr,			  // we don't have a second tracking wheel, so we set it to nullptr
	&inertial_sensor	  // inertial sensor
};
lemlib::ChassisController_t lateralController{
    15, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    30 // slew rate
};

// turning PID
ChassisController_t angularController{
	5,	// kP
	8, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500 // largeErrorTimeout
};
Chassis chassis(drivetrain, lateralController, angularController, sensors);
Controller master(pros::E_CONTROLLER_MASTER);
ASSET(skills_txt);
ASSET(red_left_txt);
ASSET(movement_txt);
void screenupdate()
{
	// loop forever
	while (true)
	{
		Pose pose = chassis.getPose();					// get the current position of the robot
		pros::lcd::print(0, "x: %f", pose.x);			// print the x position
		pros::lcd::print(1, "y: %f", pose.y);			// print the y position
		pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
		pros::delay(10);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	pros::lcd::initialize();
	chassis.calibrate();
	Task screen_task(screenupdate);
	
	//selector::init(130, 1, selector::b);
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

void update_angle(){
	while(true){
		master.print(0, 0, std::to_string(round(chassis.getPose().theta)).c_str());
		delay(100);
	}
}
void pull_back(int speed){
	if(speed > 100)
		speed = 100;
	
	if(speed < -100)
		speed = -100;
		
	while (!cata_switch.get_value()){
		cata.move( -(speed * 127) / 100);
	}
	cata.brake();
	
}
void shoot(){
	pull_back(100); // 100% backward
}
void wings(bool extend){
	left_wing.set_value(extend);
	right_wing.set_value(extend);
}
void autonomous()
{
	chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
	//switch (selector::auton)
	//{
	//	case 0:
	//		// SKILLS AUTON
	//		for (int i = 0; i < 22; i++)
	//			shoot();
	//		chassis.follow(skills_txt, 999999, 15, true, true, 127, false);
	//		chassis.waitUntilDist(12); // 12 is placeholder for now...
	//		wings(true);
	//		// EXTEND WINGS...
	//		chassis.waitUntilDist(15);
	//		wings(false);
	//		// path continues
	//		break;
	//	case 1:
	//		// RED LEFT AUTON
	//		chassis.follow(red_left_txt, 999999, 15, true, true, 127, false);
	//		chassis.waitUntilDist(12); // 12 is placeholder for now...
	//		left_wing.set_value(true); // Descore match load triball.
	//		chassis.waitUntilDist(15);
	//		left_wing.set_value(false); // Deactivate wing as it's not needed
	//		intake.move(-127); // Outtake alliance triball.
	//		chassis.waitUntilDist(17);
	//		intake.move(0); // Stop intake.
	//		break;
	//	case 2:
	//		// RED RIGHT AUTON
	//		break;
	//	case 3:
	//		// DO NOTHING
	//		break;
	//	case -3:
	//	 	// DO NOTHING AGAIN...
	//	 	break;
	//	default:
	//		break;
	//}
	chassis.turnTo(5, 0, 2000, false, false, 127, false);
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
		int forward = master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
		int turn = master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

		left_side_motors.move(forward + turn);
		right_side_motors.move(forward - turn);
		if (master.get_digital(E_CONTROLLER_DIGITAL_R1)) 
			intake.move(127);
		else if (master.get_digital(E_CONTROLLER_DIGITAL_R2))
			intake.move(-127);
		else
			intake.move(0);

		// For now I am going to code the cata to just move whenever the digital button is pressed,
		// in the future i am going to add a limit switch to have it automatically pull back
		// and have a second button for firing.

		if (master.get_digital(E_CONTROLLER_DIGITAL_L1)){
			cata.move(127);
			master.rumble(". ");
		}

		else if (master.get_digital(E_CONTROLLER_DIGITAL_L2))
			cata.move(-127);
		else
			cata.move(0);
		if(master.get_digital(E_CONTROLLER_DIGITAL_A)){
			chassis.setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
    		//chassis.moveTo(20, 20, 0, 10000, false, true, 2, 0.6, 127, false);
			//chassis.turnTo(30, 0, 6000, false, false, 127, false);
			chassis.follow(movement_txt, 10000, 1, false, true, 127, false);
		}


		pros::delay(10);
	}
}

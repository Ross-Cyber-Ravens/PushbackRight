#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp" //include the library for using pneumatics with 3-wire ports

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-10,-9},
                            pros::MotorGearset::green); // left motor group - ports 9 (reversed), 10 (reversed)
pros::MotorGroup rightMotors({2,1}, 
							pros::MotorGearset::green); // right motor group - ports 1 (reversed), 2 1 (reversed)

// Inertial Sensor on port 3
pros::Imu imu(3);

//Intake MOTOR/CONVEYOR:
pros::Motor intakeMotor(20, pros::v5::MotorGears::red); // red cartridge
//intakeMotor.set_reversed(false);   // or true if you want it reversed 

//Scoring MOTOR:
pros::Motor scoringMotor(4); 


//PNEUMATICS:
pros::adi::Pneumatics intakePiston1('A', false); //connected to port A and started retracted (false)
pros::adi::Pneumatics intakePiston2('C', false); //connected to port A and started retracted (false)

// TRACKING WHEELS
// horizontal tracking wheel encoder. Rotation sensor, port 6,  reversed
pros::Rotation horizontalEnc(-6);
// vertical tracking wheel encoder. Rotation sensor, port 5, not reversed
pros::Rotation verticalEnc(5);
// horizontal tracking wheel. 2.75" diameter, 1.5" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -1.5);
// vertical tracking wheel. 2.75" diameter, .63" offset, right of the robot (positive)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_2, .63);


 
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10.45, // 10.5 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              250, // drivetrain rpm is 250 (200rpm green cartridge * 60T gear (driving) / 48T gear (driven)
                              5 // horizontal drift is 2 if all omni wheels and 8 for all traction wheels
);


// lateral motion controller
lemlib::ControllerSettings linearController(23, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            18.6, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             9.75, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel - was &vertical
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     50, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
 void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function

ASSET(step3_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    chassis.setPose(0, 0, 0);
	
	 // lookahead distance: 10 inches, 4 second timeout
	//chassis.follow(step3_txt, 10, 4000);
	//chassis.moveToPose(15.6, 26.36, 41.94, 4000, {.maxSpeed = 120}); 

    //drive to balls
	chassis.moveToPose(15.6,26.36,41.94,4000,{.lead = 0.6,.maxSpeed = 95 });
     //spin intake motors to pick up the balls (these may need to be reversed)
    intakeMotor.move(127);
    pros::delay(500);
	//drive to small goal
	chassis.moveToPose(1.1, 36, -45, 4000); //y was 38
	pros::delay(1300);
    //spin motors out to score the balls (these may need to be reversed)
    intakeMotor.move(-127);
    scoringMotor.move(127);
    //scoringMotor.move(127); 
    pros::delay(2500);
    intakeMotor.move(0);

//back up
	chassis.moveToPoint(40.17, 3.17, 4000, {.forwards = false});
    pros::delay(50);
    chassis.turnToHeading(180, 1000, {.maxSpeed = 60});
     pros::delay(20);
    //put down intake arm
    /*intakePiston1.extend();
    intakePiston2.extend();
    //drive to tower and unload balls
    chassis.moveToPose(41.5, -12, 180, 3000,{.maxSpeed = 127});
     pros::delay(20);
     intakeMotor.move(127);
      pros::delay(3000);
      intakeMotor.move(0);
     //reverse to tall goal and score
    chassis.moveToPose(41.5, 17.7, 180, 3000,{.forwards = false, .maxSpeed = 90});
    intakeMotor.move(-127);
    scoringMotor.move(127); 
     pros::delay(5000);
      intakeMotor.move(0);
    scoringMotor.move(0); */
    
   
	
    /*
	//OR
	//Option 2:
	chassis.moveToPose( 40.17, 3.17, 0, 4000, 
    {
        .forwards = false,   // drive backward
		.horizontalDrift = 8,
		.lead = 0.3,
		.maxSpeed = 110
    });
	
	//turn to face ball tower
	chassis.turnToHeading(180, 4000,{.maxSpeed = 110}, false);// will never exceed 110, false means this motion will block execution of next line of code until finished or timedout 

	//drive to tower
	chassis.moveToPoint(40, -16, 8000, {.forwards = false}, true); //true: next line of code may run before this finishes
*/
	/*
	More Sample autonomous commands:


	// swing to head / only one side of robot turns:
	chassis.swingToHeading(45, 4000); // swing to face 45 degrees, with a timeout of 4000 ms

	//swingToPoint works exactly like swingToHeading, except it turns to face a point rather than a heading.
	chassis.swingToPoint(53, 53, 4000); // swing to face the point (53, 53) degrees, with a timeout of 4000 ms

	// Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    chassis.moveToPose(20, 15, 90, 4000);
    
	// Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    
	// cancel the movement after it has traveled 10 inches
    chassis.waitUntil(10);
    chassis.cancelMotion();
    
	// Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    
	// Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
	
	turn to 270 degrees,  // will never exceed 120 speed,  motion will not block execution
	chassis.turnToHeading(270, 4000, {.maxSpeed = 120}); 
    
	// Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    chassis.follow(example_txt, 15, 4000, false);
    
	// wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    chassis.waitUntil(10);
    pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
   
	// wait until the movement is done
    chassis.waitUntilDone();
    pros::lcd::print(4, "pure pursuit finished!");
	*/
}

/**
 * Runs in driver control
 */
void opcontrol() {

	int intakeState = 0;   // 1 = forward, -1 = reverse, 0 = stopped
    int scoringState = 0;  // 1 = forward, -1 = reverse, 0 = stopped

    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        
        chassis.curvature(leftY, rightX);

    if (controller.get_digital(DIGITAL_L1))
    {
    intakeMotor.move(115);
    pros::delay(20);
    }
    else if (controller.get_digital(DIGITAL_L2))
    {
    intakeMotor.move(-127);
    pros::delay(20);
    }
    else
    {
    intakeMotor.move(0);
    pros::delay(20);
    }

    if (controller.get_digital(DIGITAL_R1))
    {
    scoringMotor.move(127);
    pros::delay(20);
    }
    else if (controller.get_digital(DIGITAL_R2))
    {
    scoringMotor.move(-127);
    pros::delay(20);
    }
    else
    {
    scoringMotor.move(0);
    pros::delay(20);
    }


        if (controller.get_digital(DIGITAL_UP)) {
            intakePiston1.extend();
            intakePiston2.extend(); // Up button pressed → piston goes up
        } 
        if (controller.get_digital(DIGITAL_DOWN)) {
            intakePiston1.retract();
            intakePiston2.retract();   // Down button pressed → piston goes down
        }
        pros::delay(10);
    }
}
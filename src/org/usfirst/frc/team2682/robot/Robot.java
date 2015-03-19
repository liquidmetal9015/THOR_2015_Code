package org.usfirst.frc.team2682.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.Point;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;

//import edu.wpi.first.wpilibj.Servo;

public class Robot extends IterativeRobot {
	
double bias = 0;
	
	double grabR = 102;
	double grabL = -96;
	
	double narrowR = 142;
	double narrowL = -142;
	
	double restValR = narrowR;
	double restValL = narrowL;
	
	double arrowR = 85;
	double arrowL = -81;
	
	

	double wideR = 190;
	double wideL = -183;

	double stowedR = 333;
	double stowedL = -324;

	double pushR = 245;
	double pushL = -244;

	boolean lifting = false;

	boolean stowed = false;
	boolean stowedLatch = false;

	boolean wide = false;
	boolean wideLatch = false;
	
	
	
	PIDController armControlL;
	PIDController armControlR;

	VictorSP armR;
	VictorSP armL;
	
	Encoder armCodeR;
	Encoder armCodeL;
	
	
	
	// this section will act as a config section

	Timer delayTimer;

	boolean camera = true;
	boolean squaredInputs = false;
	double max_speed = 2500;
//double max_speed = 2700;
	// config section end
/*
	// To be used for recording the top speed;
	double topSpeedTestR;
	double topSpeedTestL;
	*/
	Joystick panel;

	/*
	// These should go into making the video feed function
	int session;
	Image frame;*/

	Timer time;
	// currently these value are the dead zones for the x and y
	// it seems that the best strategy going forward is to make the deadzones
	// about 0.03 with
	// the exception of the left since the joystick has about a 0.1 leeway in
	// that direction.
	// double MdeadValue = 0.1;
	// double RdeadValue = 0.1;

	double driveLeftDead = 0.06;
	double driveRightDead = 0.02;
	double driveForwardDead = 0.02;
	double driveBackwardDead = 0.07;
	
	

	// double scaleRange = 0.1;

	Joystick stick;
	Joystick auxStick;

	// These are the drive motors
	Talon motorR;
	Talon motorL;

	VictorSP lifter;

	VictorSP pullerR;
	VictorSP pullerL;
	
	//PIDController autoloopR;
	//PIDController autoloopL;

	// These are the Velocity PID controllers that regulate the drive motors.
	SuperVelocityPID controlL;
	SuperVelocityPID controlR;

	// these are the encoders on the drive train
	Encoder nCodeR;
	Encoder nCodeL;

	LifterPID lifterPID;
	AnalogInput lifterPot;

	CameraServer server;

	// These values are neccisary to the implementation of the arcadeDrive code
	// that I am using
	double moveValue;
	double rotateValue;

	double leftMotorSpeed;
	double rightMotorSpeed;

	public void robotInit() {
if(camera){
		server = CameraServer.getInstance();
		server.setQuality(50);
		// the camera name (ex "cam0") can be found through the roborio web
		// interface
		server.startAutomaticCapture("cam0");
}


//nCodeR.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
//nCodeL.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);

//autoloopL = new PIDController(-0.01,0,-0.012,nCodeR,motorR);
//autoloopR = new PIDController(-0.01,0,-0.012,nCodeL,motorL);




armL = new VictorSP(3);
armR = new VictorSP(6);

armCodeR = new Encoder(4, 5);
armCodeL = new Encoder(6, 7);

armControlR = new PIDController(0.03, 0, 0.013, armCodeR, armR);
armControlL = new PIDController(0.03, 0, 0.013, armCodeL, armL);

armControlR.enable();
armControlL.enable();

armControlR.setSetpoint(0);
armControlL.setSetpoint(0);



		/*
		 * rightServo = new Servo(7); leftServo = new Servo(8);
		 */
		nCodeL = new Encoder(0, 1);
		nCodeR = new Encoder(2, 3);

		delayTimer =  new Timer();

		stick = new Joystick(0);
		auxStick = new Joystick(1);
		panel = new Joystick(2);

		
		
		//right motor is on pwm 0, is a talon
		motorL = new Talon(0);
		//left wheel is on pwm 1, is a talon
		motorR = new Talon(1);

		
		//lifter is on 2 and is a Victor SP
		lifter = new VictorSP(2);

		//right puller is on 4, is an sp
		pullerR = new VictorSP(5);
		//left puller is 3, is an SP
		pullerL = new VictorSP(4);
		
		
		
		
		//PWM 5 IS DEAD!!!!!!!!!!!!!

		lifterPot = new AnalogInput(4);

		// This loop will control the lifter mechanism.
		lifterPID = new LifterPID(4.5, 0, 3, lifterPot, lifter);
		
		//lifterPID.setOutputRange(,0.7);

		// these are untested but are the positional PID code from my tests.
		// Thesea are for positionial use in autonomous or otehrwise.
		// autoloopL = new PIDController(-0.01,-0.00005,-0.012,nCodeR,right);
		// autoloopR = new PIDController(-0.01,-0.00005,-0.012,nCodeR,right);

		// these values seem to be working so far but I would love to have more
		// time to dial it in.
		controlL = new SuperVelocityPID(-0.0008, 0, -0.0002, nCodeL);
		controlR = new SuperVelocityPID(-0.0008, 0, -0.0002, nCodeR);

		// sets the encoders to output rate to the PID controllers.
		/*
		nCodeR.setPIDSourceParameter(PIDSourceParameter.kRate);
		nCodeL.setPIDSourceParameter(PIDSourceParameter.kRate);
		*/

		// I think this is the default output range, but I have this line just
		// incase I want to change it.
		//controlL.setOutputRange(-1, 1);
		//controlR.setOutputRange(-1, 1);

		// to start the Velocity drive PID controllers
		// I should move these to teleop init later.
		//controlL.enable();
		//controlR.enable();

		 //lifterPID.setOutputRange(-0.2, 0.2);

		lifterPID.enable();
		
		lifterPID.setSetpoint(1.8);
		
		controlL.enable();
		controlR.enable();

	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {/*
		controlL.disable();
		controlR.disable();
		
		autoloopR.setOutputRange(-0.5, 0.5);
		autoloopL.setOutputRange(-0.5, 0.5);
		autoloopR.enable();
		autoloopL.enable();
		
		
		// controlL.setSetpoint(300);
		// controlR.setSetpoint(300);
		//delayTimer.start();
*/
		armControlL.setSetpoint(-123);
		time.delay(0.15);
		armControlR.setSetpoint(128);
		

		
		//autoloopR.setSetpoint(-2500);
		//autoloopL.setSetpoint(2500);

	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {

		SmartDashboard.putNumber("lifter2", lifter.get());
		SmartDashboard.putNumber("pot Value2", lifterPot.getVoltage());
SmartDashboard.putNumber("RightArm", armCodeR.get());
SmartDashboard.putNumber("LeftArm", armCodeL.get());


		SmartDashboard
				.putNumber("Right accuracy", ((controlL.getSetpoint() - nCodeL
						.getRate()) / nCodeL.getRate()) * 100);

		SmartDashboard
				.putNumber("Left accuracy", ((controlL.getSetpoint() - nCodeL
						.getRate()) / nCodeL.getRate()) * 100);

		SmartDashboard.putNumber("LifterAccuracy2",
				((lifterPID.getSetpoint() - lifterPot.getVoltage()) / lifterPot
						.getVoltage()) * 100);

	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	public void teleopInit() {
		//autoloopR.disable();
		//autoloopL.disable();
		
		nCodeR.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate);
		nCodeL.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate);
		
		controlR.enable();
		controlL.enable();
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		/*
		 * 
		 * SmartDashboard.putNumber("Left Encoder Rate", nCodeL.getRate());
		 * SmartDashboard.putNumber("Right Motor Output", controlR.getOutput());
		 * SmartDashboard.putNumber("Left Motor Output", controlL.getOutput());
		 */
		
		SmartDashboard.putNumber("Right Arm Value", armCodeR.get());
		SmartDashboard.putNumber("Left Arm Value", armCodeL.get());
		SmartDashboard.putNumber("Right Motor Output", controlR.getOutput());
		SmartDashboard.putNumber("Left Motor Output", controlL.getOutput());
		SmartDashboard.putNumber("Right Encoder Rate", nCodeR.getRate());
		SmartDashboard.putNumber("Left Encoder Rate", nCodeL.getRate());
		SmartDashboard.putNumber("Pot value 2", lifterPot.getVoltage());
		
		
		
bias = auxStick.getX();
		
		
		if (auxStick.getRawButton(3)) 
			stowedLatch = true;

		
		if (stowedLatch && ! auxStick.getRawButton(3)) {
			stowedLatch = false;
			if (stowed) {
				stowed = false;
			} else {
				stowed = true;
			}
		}
		
		
		
		if (auxStick.getRawButton(4)) 
			wideLatch = true;

		SmartDashboard.putBoolean("wideLatch", wideLatch);
		SmartDashboard.putBoolean("wide button", stick.getRawButton(4));
		if (wideLatch && (! auxStick.getRawButton(4))) {
			wideLatch = false;
			if (restValR == narrowR) {
				restValR = wideR;
				restValL = wideL;
			} else {
				restValR = narrowR;
				restValL = narrowL; 
			}
		}

		if (auxStick.getRawButton(2)) {
			armControlR.setSetpoint(grabR+(auxStick.getX()*70) - (auxStick.getY()*40));
			armControlL.setSetpoint(grabL+(auxStick.getX()*70) + (auxStick.getY()*40));
			// stowed = false;

		} else if(auxStick.getRawButton(6)){
			armControlR.setSetpoint(arrowR+(auxStick.getX()*100) - (auxStick.getY()*30));
			armControlL.setSetpoint(arrowL+(auxStick.getX()*100) + (auxStick.getY()*30));
			//stowed = false;
			
		} else {

			if (stowed) {
				armControlR.setSetpoint(stowedR);
				armControlL.setSetpoint(stowedL);
			} else {
			armControlR.setSetpoint(restValR+(auxStick.getX()*70) - (auxStick.getY()*50));
			armControlL.setSetpoint(restValL+(auxStick.getX()*70) + (auxStick.getY()*50));
			}
		}
SmartDashboard.putNumber("right encoder Value", nCodeR.get());
SmartDashboard.putNumber("left encoder Value", nCodeL.get());
		/*
		 * 		
		 * SmartDashboard.putNumber("OutputValue", armControlR.get());
		 * SmartDashboard.putNumber("ArmR value1", armR.get());
		 * SmartDashboard.putNumber("ArmR value2", armControlR.getError());
		 * SmartDashboard.putNumber("ArmR value3", armControlR.getSetpoint());
		 */

		
		
		
		
		moveValue = stick.getY();
		rotateValue = stick.getX();

		moveValue = SuperUtils.limit(moveValue);
		rotateValue = SuperUtils.limit(rotateValue);

		if (panel.getRawButton(6) || auxStick.getRawButton(1)) {
			lifterPID.setSetpoint(3.68);
			//3.6
			lifting = true;
		} else if (panel.getRawButton(1)){
			lifterPID.setSetpoint(1.902);
			//1.95
			lifting = false;
		} else {
			lifterPID.setSetpoint(1.504);
			//1.53
			lifting = false;
		}
		

		//picking up height
		//1.504
		//
		
		//latching height
		//3.505
		
		
		//drop-off Height
		//1.902
		
//1.93
//1.996
		// lifterPID.setSetpoint((Math.abs(auxStick.getZ())*2.425)+2.4);
		// lifter.set(auxStick.getX());

		if(panel.getRawButton(11) || lifting == true){
			pullerR.set(-1+((auxStick.getThrottle()+1)/2));
			pullerL.set(1-((auxStick.getThrottle()+1)/2));
		} else if (panel.getRawButton(8)) {
			pullerR.set((1-((auxStick.getThrottle()+1)/2)));
			pullerL.set(-1+((auxStick.getThrottle()+1)/2));
		}  else {
			pullerR.set(0);
			pullerL.set(0);
		}

		if (squaredInputs) { // square the inputs (while preserving the sign to
								// increase fine control while permitting full
								// power
			if (moveValue >= 0.0) {
				moveValue = (moveValue * moveValue);
			} else {
				moveValue = -(moveValue * moveValue);
			}
			if (rotateValue >= 0.0) {
				rotateValue = (rotateValue * rotateValue);
			} else {
				rotateValue = -(rotateValue * rotateValue);
			}
		}

		if (moveValue > 0.0) {
			if (rotateValue > 0.0) {
				leftMotorSpeed = moveValue - rotateValue;
				rightMotorSpeed = Math.max(moveValue, rotateValue);
			} else {
				leftMotorSpeed = Math.max(moveValue, -rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			}
		} else {
			if (rotateValue > 0.0) {
				leftMotorSpeed = -Math.max(-moveValue, rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			} else {
				leftMotorSpeed = moveValue - rotateValue;
				rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
			}
		}

		if (squaredInputs) {
			if (leftMotorSpeed >= 0.0) {
				leftMotorSpeed = (leftMotorSpeed * leftMotorSpeed);
			} else {
				leftMotorSpeed = -(leftMotorSpeed * leftMotorSpeed);
			}
			if (rightMotorSpeed >= 0.0) {
				rightMotorSpeed = (rightMotorSpeed * rightMotorSpeed);
			} else {
				rightMotorSpeed = -(rightMotorSpeed * rightMotorSpeed);
			}
		}

		if (stick.getRawButton(1)) {
			// squaredInputs = true;
			controlL.setSetpoint(leftMotorSpeed * 1000);
			controlR.setSetpoint(rightMotorSpeed * -1000);

		} else if (stick.getRawButton(2)) {
			// squaredInputs = false;
		    controlL.setSetpoint(leftMotorSpeed * max_speed);
		    controlR.setSetpoint(rightMotorSpeed * -max_speed);

		} else {
			// squaredInputs = false;
			controlL.setSetpoint(leftMotorSpeed * max_speed);
			controlR.setSetpoint(rightMotorSpeed * -max_speed);
			// squaredInputs = true;
						controlL.setSetpoint(leftMotorSpeed * 500);
						controlR.setSetpoint(rightMotorSpeed * -500);

		}

		SmartDashboard
				.putNumber("Right accuracy", ((controlL.getSetpoint() - nCodeL
						.getRate()) / nCodeL.getRate()) * 100);

		SmartDashboard
				.putNumber("Left accuracy", ((controlL.getSetpoint() - nCodeL
						.getRate()) / nCodeL.getRate()) * 100);

		SmartDashboard.putNumber("LifterAccuracy",
				((lifterPID.getSetpoint() - lifterPot.getVoltage()) / lifterPot
						.getVoltage()) * 100);

		if (moveValue > driveBackwardDead)
			moveValue = moveValue - driveBackwardDead;
		else if (moveValue < -driveForwardDead)
			moveValue = moveValue + driveForwardDead;

		if (rotateValue >= driveLeftDead)
			rotateValue = rotateValue - driveLeftDead;
		else if (rotateValue < -driveLeftDead)
			rotateValue = rotateValue + driveLeftDead;

		/*
		 * SmartDashboard.putNumber("driveValue", moveValue);
		 * SmartDashboard.putNumber("rotateValue", rotateValue);
		 * SmartDashboard.putNumber("leftValue", leftMotorSpeed);
		 * SmartDashboard.putNumber("rightValue", rightMotorSpeed);
		 * SmartDashboard.putNumber("potValue omega", lifterPot.pidGet());
		 */

		if (((moveValue < driveBackwardDead) && (moveValue > -driveForwardDead))
				&& (rotateValue < driveLeftDead)
				&& (rotateValue > -driveRightDead)) {

			motorR.set(0);
			motorL.set(0);
			SmartDashboard.putBoolean("stopped", true);

		} else {

			motorR.set(controlR.getOutput());
			motorL.set(controlL.getOutput());
			SmartDashboard.putBoolean("stopped", false);
		}

		// Top speed test code
		/*
		 * if (Math.abs(nCodeR.getRate()) > topSpeedTestR) { topSpeedTestR =
		 * Math.abs(nCodeR.getRate());
		 * SmartDashboard.putNumber("right top speed", topSpeedTestR); }
		 * 
		 * if (Math.abs(nCodeL.getRate()) > topSpeedTestL) { topSpeedTestL =
		 * Math.abs(nCodeL.getRate());
		 * SmartDashboard.putNumber("left top speed", topSpeedTestL); }
		 */
	}

	/*
	 * WARNING!!! In its current state the test code is designed to be used in the pits only
	 * The robot code must be reset in order for the robot to function after the 
	 */
	
	
	public void testPeriodic() {
		lifter.set(auxStick.getY());
	}
	
	public void testInit() {
		controlL.disable();
		controlR.disable();
		lifterPID.disable();
		//3.622
		
		/*
		SmartDashboard.putBoolean("The Pot is not working", false );
		SmartDashboard
		.putBoolean(
				"The lifter appears to be going in the wrong direction",
				false);
		SmartDashboard
		.putBoolean("The Lifter is taking too long", false);
		
		
		Timer tooLong = new Timer();
		
		boolean lifterIsFine = true;
		boolean wheelsAreFine = true;
		double close = 999;
		double closeLast = 999;

		// Lifter diagnostics start here ---------------------
		if ((lifterPot.getVoltage() < 1.9) || (lifterPot.getVoltage() > 4.5)) {
			SmartDashboard.putBoolean("The Pot is not working", true);
			lifterIsFine = false;
		}
		lifterPID.enable();
		lifterPID.setSetpoint(2.5);
		while (!lifterPID.onTarget() && lifterIsFine) {
			tooLong.start();
			if (Math.abs(lifterPot.getVoltage() - 2.5) < close) {
				closeLast = close;
				close = Math.abs(lifterPot.getVoltage() - 2.5);

			}
			time.delay(0.3);
			if (close > closeLast) {

				lifterIsFine = false;
				lifterPID.disable();
				lifter.set(0);
				SmartDashboard
						.putBoolean(
								"The lifter appears to be going in the wrong direction",
								true);
			}
			if (tooLong.get() > 3) {
				lifterIsFine = false;
				lifterPID.disable();
				lifter.set(0);
				SmartDashboard
						.putBoolean("The Lifter is taking too long", true);
			}
			SmartDashboard.putNumber("LifterError", ((lifterPID.getSetpoint() - lifterPot.getVoltage()) / lifterPot
					.getVoltage()) * 100);
		}
lifterPID.disable();
lifter.set(0);
time.delay(2);

//Wheels test begins here
if(wheelsAreFine)
motorR.set(0.3);
time.delay(1);
		
		
		
		
	}
*/
	

		

	}
}
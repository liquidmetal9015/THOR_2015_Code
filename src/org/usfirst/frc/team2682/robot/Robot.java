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
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Timer;

//import edu.wpi.first.wpilibj.Servo;

public class Robot extends IterativeRobot {
	// this section will act as a config section

	Timer delayTimer;

	boolean camera = false;
	boolean squaredInputs = false;
	double max_speed = 2500;

	// config section end

	// To be used for recording the top speed;
	double topSpeedTestR;
	double topSpeedTestL;

	// These should go into making the video feed function
	int session;
	Image frame;

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

	Talon lifter;

	TalonSRX pullerR;
	TalonSRX pullerL;

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

		server = CameraServer.getInstance();
		server.setQuality(50);
		// the camera name (ex "cam0") can be found through the roborio web
		// interface
		server.startAutomaticCapture("cam0");

		/*
		 * rightServo = new Servo(7); leftServo = new Servo(8);
		 */
		nCodeL = new Encoder(0, 1);
		nCodeR = new Encoder(2, 3);

		delayTimer = new Timer();

		stick = new Joystick(0);
		auxStick = new Joystick(1);

		motorL = new Talon(2);
		motorR = new Talon(0);

		lifter = new Talon(1);

		pullerR = new TalonSRX(3);
		pullerL = new TalonSRX(4);

		lifterPot = new AnalogInput(4);

		// This loop will control the lifter mechanism.
		lifterPID = new LifterPID(4.5, 0, 3, lifterPot, lifter);

		// these are untested but are the positional PID code from my tests.
		// Thesea are for positionial use in autonomous or otehrwise.
		// autoloopL = new PIDController(-0.01,-0.00005,-0.012,nCodeR,right);
		// autoloopR = new PIDController(-0.01,-0.00005,-0.012,nCodeR,right);

		// these values seem to be working so far but I would love to have more
		// time to dial it in.
		controlL = new SuperVelocityPID(-0.0008, 0, -0.0002, nCodeL);
		controlR = new SuperVelocityPID(-0.0008, 0, -0.0002, nCodeR);

		// sets the encoders to output rate to the PID controllers.
		nCodeR.setPIDSourceParameter(PIDSourceParameter.kRate);
		nCodeL.setPIDSourceParameter(PIDSourceParameter.kRate);

		// I think this is the default output range, but I have this line just
		// incase I want to change it.
		controlL.setOutputRange(-1, 1);
		controlR.setOutputRange(-1, 1);

		// to start the Velocity drive PID controllers
		// I should move these to teleop init later.
		controlL.enable();
		controlR.enable();

		// lifterPID.setOutputRange(-0.2, 0.2);

		lifterPID.enable();

	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
		// controlL.setSetpoint(300);
		// controlR.setSetpoint(300);
		delayTimer.start();

	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {

		SmartDashboard.putNumber("lifter", lifter.get());
		SmartDashboard.putNumber("pot Value", lifterPot.getVoltage());

	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	public void teleopInit() {

	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		/*
		 * SmartDashboard.putNumber("Right Encoder Rate", nCodeR.getRate());
		 * SmartDashboard.putNumber("Left Encoder Rate", nCodeL.getRate());
		 * SmartDashboard.putNumber("Right Motor Output", controlR.getOutput());
		 * SmartDashboard.putNumber("Left Motor Output", controlL.getOutput());
		 */
		moveValue = stick.getY();
		rotateValue = -stick.getX();

		moveValue = SuperUtils.limit(moveValue);
		rotateValue = SuperUtils.limit(rotateValue);

		if (auxStick.getRawButton(1)) {
			lifterPID.setSetpoint(3.95);
		} else {
			lifterPID.setSetpoint(1.87);

		}

		// lifterPID.setSetpoint((Math.abs(auxStick.getZ())*2.425)+2.4);
		// lifter.set(auxStick.getX());

		pullerR.set(-auxStick.getY());
		pullerL.set(auxStick.getY());

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
			controlL.setSetpoint(leftMotorSpeed * -1000);
			controlR.setSetpoint(rightMotorSpeed * 1000);

		} else if (stick.getRawButton(2)) {
			// squaredInputs = true;
			controlL.setSetpoint(leftMotorSpeed * -500);
			controlR.setSetpoint(rightMotorSpeed * 500);

		} else {
			// squaredInputs = false;
			controlL.setSetpoint(leftMotorSpeed * -max_speed);
			controlR.setSetpoint(rightMotorSpeed * max_speed);

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
	public void testInit() {
		controlL.disable();
		controlR.disable();
		lifterPID.disable();
		
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
if		
		
		
		
	}

	public void testPeriodic() {

	}

}
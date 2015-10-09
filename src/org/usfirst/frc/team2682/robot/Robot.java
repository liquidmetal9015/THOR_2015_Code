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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DigitalInput;

//import edu.wpi.first.wpilibj.Servo;

public class Robot extends IterativeRobot {
	//This note is a test
	
Compressor compress;	
	

DigitalInput armRLimit;
DigitalInput armLLimit;


	

double armOpen = 10;
double armClosed = 15;
double armScore = 10;
double armIn = 20;

double stepClose = 965;

boolean ejecting = false;
boolean score = false;
boolean sucking = false;
boolean lifting = false;

double rackHeight = 3.72;
double grabHeight = 1.3;
//1.453
double scoreHeight = 1.902;
double stepHeight = 2.65;



	

	
	
	boolean wide = false;
	boolean wideLatch = false;
	
	Timer liftStall;
	Timer liftStall2;
	Timer liftTimeout;
	
	DigitalInput armZeroR;
	DigitalInput armZeroL;
	
	boolean liftSeqLatch = false;
	boolean liftSeqLift = false;
	boolean liftSeqTop = false;
	boolean liftSeqStall = false;
	boolean liftSeqStall2 = false;
	boolean liftSeqDrop = false;
	
	
	//boolean armsOpen = false;
	//boolean armsOpenLatch = false;
	//ARMS CLOSED DOES NOT DO ANYTHING RIGHT NOW
	boolean armsClosed = false;
	//boolean armsClosedLatch = false;
	boolean armsIn = false;
	boolean armsInLatch = false;
	//boolean lifterHigh = false;
	
	
	
	//boolean wheelsIn = false;
	//boolean wheelsOut = false;
	
	boolean clamped = false;
	DoubleSolenoid clamp; 
	
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
	double max_speed = 2700;

	//Joystick panel;

	

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
	
	

	

	Joystick stick;
	Joystick auxStick;

	
	Talon motorR;
	Talon motorL;

	VictorSP lifter;

	VictorSP pullerR;
	VictorSP pullerL;
	
	

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


clamp = new DoubleSolenoid(0,1);

//nCodeR.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
//nCodeL.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);

//autoloopL = new PIDController(-0.01,0,-0.012,nCodeR,motorR);
//autoloopR = new PIDController(-0.01,0,-0.012,nCodeL,motorL);

//armZeroR = new DigitalInput();
//armZeroL = new DigitalInput();

	liftStall = new Timer();
	liftStall2 = new Timer();
	liftTimeout = new Timer();


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


armRLimit = new DigitalInput(8);
armLLimit = new DigitalInput(9);


		/*
		 * rightServo = new Servo(7); leftServo = new Servo(8);
		 */
		nCodeL = new Encoder(0, 1);
		nCodeR = new Encoder(2, 3);

		delayTimer =  new Timer();

		stick = new Joystick(0);
		auxStick = new Joystick(1);
		//panel = new Joystick(2);

		
		
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
		
		
		
		
		
		lifterPot = new AnalogInput(4);

		// This loop will control the lifter mechanism.
		lifterPID = new LifterPID(4.5, 0, 3, lifterPot, lifter);
		
		
		// these values seem to be working so far but I would love to have more
		// time to dial it in.
		controlL = new SuperVelocityPID(-0.0008, 0, -0.0002, nCodeL);
		controlR = new SuperVelocityPID(-0.0008, 0, -0.0002, nCodeR);

	
	//lifterPID.setOutputRange(-0.5, 0.5);

		

		
		
		//THIS IS A SAFETY PRECAUSION// NEEDS TO BE DELETED
		
				//armControlR.setOutputRange(-0.3, 0.3);
				//armControlL.setOutputRange(-0.3, 0.3);

		lifterPID.enable();
		
		lifterPID.setSetpoint(grabHeight);
		
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
		
		//nCodeR.reset();
		//nCodeL.reset();

		
		//autoloopR.setSetpoint(-2500);
		//autoloopL.setSetpoint(2500);
controlL.disable();
controlR.disable();



//armControlR.disable();
//armControlL.disable();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {/*

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
						.getVoltage()) * 100);*/
		
		
		
		if(!armRLimit.get()){
			armR.set(0.2);
		} else {
			armR.set(0);
			armCodeR.reset();
		}
			
		if(!armLLimit.get()){
			armR.set(-0.2);
		} else {
			armL.set(0);
			armCodeL.reset();
		}
		
		
		if(false/*(Math.abs(nCodeR.get()) < 2500 &&  Math.abs(nCodeL.get()) < 2500)/* && ! panel.getRawButton(3)*/){
			motorR.set(-0.5);
			motorL.set(0.5);
		} else {
			motorR.set(0);
			motorL.set(0);
			armControlR.setSetpoint(armOpen);
			armControlL.setSetpoint(-armOpen);
			
		}
		
		
		
		
		
		
		
		
		/*if(! armZeroR.get()){
			armR.set(0.2);
		} else {
			armR.set(0);
			armCodeR.reset();
		}
		
		if(! armZeroL.get()){
			armL.set(-0.2);
		} else {
			armL.set(0);
			armCodeL.reset();
		}*/
		
		
		/*
		SmartDashboard.putNumber("nCode 234R", nCodeR.get());
		SmartDashboard.putNumber("ControlL setpoint 345", controlL.getSetpoint());
		SmartDashboard.putNumber("ControlR setpoint 567", controlR.getSetpoint());
		SmartDashboard.putNumber("nCodeL 57", nCodeL.get());
		SmartDashboard.putNumber(" MotorR 9786", motorR.get());
		SmartDashboard.putNumber("MotorL 567 ", motorL.get());
		SmartDashboard.putBoolean("done? ", done);*/
		
		/*
		if(done){
			armControlL.setSetpoint(-123);
			time.delay(0.15);
			armControlR.setSetpoint(128);
			
		}*/

	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	public void teleopInit() {
		clamp.set(Value.kForward);
		//autoloopR.disable();
		//autoloopL.disable();
		
		nCodeR.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate);
		nCodeL.setPIDSourceParameter(PIDSource.PIDSourceParameter.kRate);
		
		
		
		armControlR.enable();
		armControlL.enable();
		controlR.enable();
		controlL.enable();
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		/*
		if(stick.getRawButton(4)){
			controlL.disable();
			controlR.disable();
			
			motorR.set(0.5);
			motorL.set(0.5);
		} else if (stick.getRawButton(5)) {
			control
		} else {
			controlL.enable();
			controlR.enable();
			
		}*/
		
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
		
		
		
//bias = auxStick.getX();
		
		

		
		//ARMS IN LATCH CONTROL
		/*if (auxStick.getRawButton(3)) {
			armsInLatch = true;
		}
		
		if (armsInLatch && ! auxStick.getRawButton(3)) {
			armsInLatch = false;
			if(armsIn)
			armsIn = false;
			else 
			armsIn = true;	
		}*/
		SmartDashboard.putBoolean("lifting", lifting);
		SmartDashboard.putBoolean("scoring", score);
		SmartDashboard.putBoolean("sucking", sucking);
		SmartDashboard.putBoolean("ejecting", ejecting);
		SmartDashboard.putNumber("pot Value 234", lifterPot.getVoltage());
		
		SmartDashboard.putNumber("pot Target", lifterPID.getSetpoint());
		
		SmartDashboard.putBoolean("liftSeq lift", liftSeqLift);
		SmartDashboard.putBoolean("liftSeq latch", liftSeqLatch);
		SmartDashboard.putBoolean("liftSeq top", liftSeqTop);
		SmartDashboard.putBoolean("liftSeq stall", liftSeqStall);
		SmartDashboard.putBoolean("liftSeq stall 2", liftSeqStall2);
		SmartDashboard.putBoolean("liftSeq drop", liftSeqDrop);
		
		
		/*boolean liftSeqLatch = false;
	boolean liftSeqLift = false;
	boolean liftSeqTop = false;
	boolean liftSeqStall = false;
	boolean liftSeqStall2 = false;
	boolean liftSeqDrop = false;*/
		
		//SmartDashbaord.putNumber("");
		
		//ARMS CONTROL STRUCTURE
	if(lifting){
			armControlR.setSetpoint(armOpen+50);
			armControlL.setSetpoint(-armOpen);
	} else if (lifterPID.getSetpoint() == stepHeight) {
		armControlR.setSetpoint(stepClose+50);
		armControlL.setSetpoint(-stepClose);
		} else if ((ejecting && score) || (sucking && score)) {
			armControlR.setSetpoint(armScore+50);
			armControlL.setSetpoint(-armScore);
	
		}else if(/*armsClosed || */sucking || ejecting){
			armControlR.setSetpoint(armClosed+50);
			armControlL.setSetpoint(-armClosed);
		} else if(armsIn){
			armControlR.setSetpoint(armIn+50);
			armControlL.setSetpoint(-armIn);
		} else {
			armControlR.setSetpoint(armOpen+50);
			armControlL.setSetpoint(-armOpen);
		}
        
     
        
		//THIS BIT MAKES THE ROBOT GO
		moveValue = stick.getY();
		rotateValue = stick.getX();

		moveValue = SuperUtils.limit(moveValue);
		rotateValue = SuperUtils.limit(rotateValue);
		
		
		//THIS IS THE BEGINNING OF THE LIFT SEQUENCE
		if(auxStick.getRawButton(8)){
			 liftSeqLatch = true;
			 liftSeqLift = false;
			 liftSeqTop = false;
			 liftSeqDrop = false;
			 
			 liftTimeout.stop();
				liftTimeout.reset();
				
				liftStall.stop();
				liftStall.reset();
				
				liftStall2.stop();
				liftStall2.reset();
			
			
		}
//THIS IS THE FIRST STEP // CHANGE SETPOINT TO LATCH HEIGHT
if(liftSeqLatch && ! auxStick.getRawButton(8)){
	liftSeqLatch = false;
	 liftSeqLift = true;
	 liftTimeout.reset();
	 liftTimeout.start();
}

//WHEN THE HEIGHT REACHES CLOSE TO THE TOP OR 3 SECONDS EXPIRES BEGIN A TIMER TO DELAY THE CLAMP
if((liftSeqLift && lifterPot.getVoltage() > 3.3) || (liftTimeout.get() > 3 && liftSeqLift)){
	liftSeqLift = false;
	liftTimeout.stop();
	liftTimeout.reset();
	liftSeqStall = true;
	liftStall.start();
	
}
//DELAY FOR A TIME BEFORE ENGAGING CLAMP
if(liftStall.get() > 0.20){
	liftSeqStall = false;
	liftStall.stop();
	liftStall.reset();
	liftStall2.start();
	liftSeqStall2 = true;
	
	
}
//ENGAGE CLAMP AND WAIT A FURTHER TIME
if(liftStall2.get() > 0.30){
	liftSeqStall2 = false;
	liftSeqDrop = true;
	liftStall2.stop();
	liftStall2.reset();
}
//SET THE LIFTER SETPOINT BACK TO DEFAULT// TURNING LIFT SEQ DROP OFF IS JUST A FORMALITY
if(liftSeqDrop && lifterPot.getVoltage() < 1.75){
	liftSeqDrop = false;
	
	
}


if(liftSeqLift || liftSeqStall || score || auxStick.getRawButton(9) || auxStick.getRawButton(7) || auxStick.getRawButton(11)){
	clamp.set(Value.kReverse);
	
} else {
	clamp.set(Value.kForward);
	
}



//IF A BUTTON ON THE PANEL IS PRESSED OR THE LIFT SEQUENCE IS IN THE LIFTING, STALL1, OR STALL2
//SECTIONS, KEEP THE LIFT AT THE RACKING HEIGHT
		if (/*auxStick.getRawButton(7) || */liftSeqLift || liftSeqStall || liftSeqStall2) {
			lifterPID.setSetpoint(rackHeight);
			//3.68
			lifting = true;
			//THE STEP HEIGHT IS ON MANUAL CONTROL AT THIS TIME
		} else if(auxStick.getRawButton(7)){
			lifterPID.setSetpoint(stepHeight);
			lifting = false;
			//2.63
			//THE SCORE HEIGHT IS ON MANUAL CONTROL AT THIS TIME
		} else if (auxStick.getRawButton(11)){
			lifterPID.setSetpoint(scoreHeight);
			//1.902
			lifting = false;
			//GRAB HEIGHT IS THE DEFAULT IF NONE OF THE ABOVE CONDITIONS ARE MET
		} else {
			lifterPID.setSetpoint(grabHeight);
			//1.504
			lifting = false;
		}
		

		//picking up height
		//1.504
		//
		
		//latching height
		//3.505
		
		
		//drop-off Height
		//1.902
		

		
//THIS IS FOR THE UNUSUAL CONDITION OF SUCKING TOTES OFF OF THE SCORRING PLATFROM		
//IF THE INTAKE BUTTON IS DEPRESSED AND THE SCORE HEIGHT IS THE LIFTER TARGET,
//THE ARMS SHOULD CHANGE TO A NARROWER CONFIGURATION AND THE PULLER WILL SUCK IN THE TOTE
		/*if(auxStick.getRawButton(11) && lifterPID.getSetpoint() == scoreHeight){
			pullerR.set(-1+((auxStick.getThrottle()+1)/2));
			pullerL.set(1-((auxStick.getThrottle()+1)/2));
			sucking = true;
			score = true;
			//THIS IS THE DEFAULT CASE FOR TOTE INTAKE
		} else */if (auxStick.getRawButton(10)){
			pullerR.set(-1+((auxStick.getThrottle()+1)/2));
			pullerL.set(1-((auxStick.getThrottle()+1)/2));
			sucking = true;
			score = false;
			ejecting = false;
			//IF THE LIFTER IS AT EJECTION HEIGHT THE ARMS SHOULD NARROW TO GRAB AND EJECT THE TOTE
	
			
	
/*}else if (auxStick.getRawButton(10) && lifterPID.getSetpoint() == scoreHeight) {
			pullerR.set((1-((auxStick.getThrottle()+1)/2)));
			pullerL.set(-1+((auxStick.getThrottle()+1)/2));
			score = true;
			sucking = false;
			ejecting = true;*/
		}else if (auxStick.getRawButton(12)) {
			//THIS IS THE DEFAULT CASE FOR  EJECTION
			pullerR.set((1-((auxStick.getThrottle()+1)/2)));
			pullerL.set(-1+((auxStick.getThrottle()+1)/2));
			ejecting = true;
			score = false;
			sucking = false;
			//THIS IS THE DEFAULT STATE FOR THE PULLERS
		}  else {
			pullerR.set(0);
			pullerL.set(0);
			
			score = false;
			sucking = false;
			ejecting = false;
		}

		
	 
		//IF THE LIFTER IS AT EJECTION HEIGHT THE ARMS SHOULD NARROW TO GRAB AND EJECT THE TOTE
		//THIS IS AN OPTIONAL CONTROL MODE THAT WE DON'T REALLY USE EVER
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

		//THIS LOGIC INTERFACED THE JOYSTICK WITH THE WHEEL SPEEDS TO CREATE ARCADE DRIVE
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
//THIS IS THE SECOND COMPONENT OF THE SQUARED INPUT FEATURE
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

		
		//THESE ARE THE SPEED SCALING SETTINGS AND THE CODE THAT SETS THE WHEEL SPEED
		//THIS IS MEDIUM SPEED
		if (stick.getRawButton(1)) {
			// squaredInputs = true;
			controlL.setSetpoint(leftMotorSpeed * 1000);
			controlR.setSetpoint(rightMotorSpeed * -1000);
//THIS IS MAXIMUM SPEE D
		} else if (stick.getRawButton(2)) {
			// squaredInputs = false;
		    controlL.setSetpoint(leftMotorSpeed * max_speed);
		    controlR.setSetpoint(rightMotorSpeed * -max_speed);
//THIS IS THE SLOWEST SPEED
		} else if (stick.getRawButton(3)) {
			controlL.setSetpoint(leftMotorSpeed * 250);
		    controlR.setSetpoint(rightMotorSpeed * -250);
			//I DON'T KNOW IF THESE BUTTONS ACTUALLY WORK BUT THEY SHOULD SLOWELY ROTATE THE ROBOT
		} else if (stick.getRawButton(5)) {
			controlL.setSetpoint(50);
			controlR.setSetpoint(50);
		} else if (stick.getRawButton(4)) {
			controlL.setSetpoint(-50);
			controlR.setSetpoint(-50);
			
			//OUR DEFAULT SPEED IS 500
		} else {
			
		
						controlL.setSetpoint(leftMotorSpeed * 500);
						controlR.setSetpoint(rightMotorSpeed * -500);

		}
		
		//THIS DEBUG BLOCK SHOWS THE PERCENT ERROR IN WHEEL VELOCITY AND LIFTER POSITION.
/*
		SmartDashboard
				.putNumber("Right accuracy", ((controlL.getSetpoint() - nCodeL
						.getRate()) / nCodeL.getRate()) * 100);

		SmartDashboard
				.putNumber("Left accuracy", ((controlL.getSetpoint() - nCodeL
						.getRate()) / nCodeL.getRate()) * 100);

		SmartDashboard.putNumber("LifterAccuracy",
				((lifterPID.getSetpoint() - lifterPot.getVoltage()) / lifterPot
						.getVoltage()) * 100);
*/
		
		//THIS CODE GETS RID OF THE JITTER WHEN MOVING OUT OF THE DEADZONE
		//KEEP IN MIND THIS TECHNICALLY REDUCES OUR MAXIMUM SPEED BY UP TO 7%
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

		//THIS IS THE ACTUALLY THE DEADZONE CODE
		//I SHOULD CONSIDER THE IMPLICATIONS OF THE ORDER OF THIS CODE, MIGHT BE NOTHING
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

	
	public void testPeriodic() {
		SmartDashboard.putNumber("Right Arm Value", armCodeR.get());
		SmartDashboard.putNumber("Left Arm Value", armCodeL.get());
		SmartDashboard.putNumber("Right Motor Output", controlR.getOutput());
		SmartDashboard.putNumber("Left Motor Output", controlL.getOutput());
		SmartDashboard.putNumber("Right Encoder Rate", nCodeR.getRate());
		SmartDashboard.putNumber("Left Encoder Rate", nCodeL.getRate());
		SmartDashboard.putNumber("Pot value 2", lifterPot.getVoltage());
	}
	
	public void testInit() {
		controlL.disable();
		controlR.disable();
		
		armControlR.disable();
		armControlL.disable();
		
		lifterPID.disable();
		/*controlL.disable();
		controlR.disable();
		lifterPID.disable();*/
		
		
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
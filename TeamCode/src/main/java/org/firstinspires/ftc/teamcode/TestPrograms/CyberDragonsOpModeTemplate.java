/*
 make a copy of this file and use it as a template to start your programming;
 initialization and all hardware components are done, just start creating
 your opmode, whether it be autonomous or teleop
 */

package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

//uncomment the mode, whether it be tele-op or autonomous
//@TeleOp
//@Autonomous
public class CyberDragonsOpModeTemplate extends LinearOpMode {

	// DO NOT MAKE PUBLIC
	//Motors
	private DcMotorEx frontRight;
	private DcMotorEx frontLeft;
	private DcMotorEx backLeft;
	private DcMotorEx backRight;

	//Arm + Other Motors
	private DcMotorEx bucket;
	private DcMotorEx bucketTurner;
	private DcMotorEx armMotor;
	private DcMotorEx carouselTurner;

	//Servos
	private Servo intakeServo;

	//Sensors
	private DistanceSensor distanceLeftFront;
	private DistanceSensor distanceLeftBack;
	private DistanceSensor distanceRightFront;
	private DistanceSensor distanceRightBack;
	private DistanceSensor distanceFront;
	private DistanceSensor distanceIntake;
	private ColorSensor colorFront;

	// Variables for the IMU
	BNO055IMU			   imu;
	Orientation			 lastAngles = new Orientation();
	double				  globalAngle, correction;

	@Override
	public void runOpMode() throws InterruptedException {


		initializeRobot();

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		// Wait for the game to start (driver presses PLAY)
		waitForStart();

		// run until the end of the match (driver presses STOP)
		//while (opModeIsActive())
		if (opModeIsActive()) {
			//code here to run during the opmode, note that it is a while loop


		}
	}

	// add functions here
	public void initializeRobot() {

		// initializing drive wheels
		frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
		frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
		backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
		backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

		// initializing other motors
		bucket = hardwareMap.get(DcMotorEx.class, "Bucket");
		bucketTurner = hardwareMap.get(DcMotorEx.class, "BucketTurner");
		armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
		carouselTurner = hardwareMap.get(DcMotorEx.class, "CarouselTurner");

		//initialize servo
		intakeServo = hardwareMap.get(Servo.class, "intakeServo");

		// initializing sensors
		distanceLeftFront = hardwareMap.get(DistanceSensor.class, "distanceLeftFront");
		distanceLeftBack = hardwareMap.get(DistanceSensor.class, "distanceLeftBack");
		distanceRightFront = hardwareMap.get(DistanceSensor.class, "distanceRightFront");
		distanceRightBack = hardwareMap.get(DistanceSensor.class, "distanceRightBack");
		distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
		distanceIntake = hardwareMap.get(DistanceSensor.class, "distanceIntake");
		colorFront = hardwareMap.get(ColorSensor.class, "colorFront");
		//initializeIMU() ;

		// sets the motors to use encoders
		frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		bucketTurner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
		carouselTurner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

		//reset encoders
		frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

		armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		bucketTurner.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
		carouselTurner.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

		// sets the motors to brake when there is no power applied (motor tries to actively maintain position)
		frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		bucket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		bucketTurner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		carouselTurner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		//set direction of drive motors
		frontLeft.setDirection(DcMotor.Direction.FORWARD);
		frontRight.setDirection(DcMotor.Direction.REVERSE);
		backLeft.setDirection(DcMotor.Direction.FORWARD);
		backRight.setDirection(DcMotor.Direction.REVERSE);

		//set direction of other motors
		armMotor.setDirection(DcMotor.Direction.FORWARD);
		bucketTurner.setDirection(DcMotor.Direction.REVERSE);
		bucket.setDirection(DcMotor.Direction.FORWARD);
		carouselTurner.setDirection(DcMotor.Direction.FORWARD);

		telemetry.addData("Status", "Initialized") ;
		telemetry.update() ;
	}

	public void moveBucketToEncoderVal(int encValBuc, double pval) {

		bucketTurner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		bucketTurner.setTargetPosition(encValBuc);

		bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		bucketTurner.setPower(pval);

		while (bucketTurner.isBusy()) {


		}

	}

	public void moveArmToEncoderVal(int encValArm, double pval) {

		armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		armMotor.setTargetPosition(encValArm);

		armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		armMotor.setPower(pval);

		while (armMotor.isBusy()) {


		}

	}

	public void moveArmWithBucket(int encValArm, double pvalArm, int encValBuc, double pvalBuc) {


			armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			bucketTurner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

			armMotor.setTargetPosition(encValArm);
			bucketTurner.setTargetPosition(encValBuc);

			armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);

			armMotor.setPower(pvalArm);
			bucketTurner.setPower(pvalBuc);

			while (armMotor.isBusy() || bucketTurner.isBusy()) {


			}

	}

  public void stopDriveMotors() {
	frontRight.setVelocity(0) ;
	frontLeft.setVelocity(0) ;
	backRight.setVelocity(0) ;
	backLeft.setVelocity(0) ;
  }

  public void moveForwardVelocity(double nticks) {
	frontRight.setVelocity(nticks) ;
	frontLeft.setVelocity(nticks) ;
	backRight.setVelocity(nticks) ;
	backLeft.setVelocity(nticks) ;
  }

  public void moveBackwardVelocity(double nticks) {
	frontRight.setVelocity(nticks * -1.0) ;
	frontLeft.setVelocity(nticks * -1.0) ;
	backRight.setVelocity(nticks * -1.0) ;
	backLeft.setVelocity(nticks * -1.0) ;
  }

  public void turnRightVelocity(double nticks) {
	frontLeft.setVelocity(nticks) ;
	backLeft.setVelocity(nticks) ;
	frontRight.setVelocity(0) ;
	backRight.setVelocity(0) ;
  }

  public void turnLeftVelocity(double nticks) {
	frontRight.setVelocity(nticks) ;
	backRight.setVelocity(nticks) ;
	frontLeft.setVelocity(0) ;
	backLeft.setVelocity(0) ;
  }

  public void strafeLeftVelocity(double nticks) {
	frontRight.setVelocity(nticks) ;
	backLeft.setVelocity(nticks) ;
	backRight.setVelocity(nticks * -1.0) ;
	frontLeft.setVelocity(nticks * -1.0) ;
  }

  	public void strafeRightVelocityforauto(double nticks) {
        frontRight.setVelocity(nticks * -0.9) ;
        backLeft.setVelocity(nticks * -0.9) ;
        backRight.setVelocity(nticks) ;
        frontLeft.setVelocity(nticks) ;

    }

	public void strafeRightVelocity(double nticks) {
		frontRight.setVelocity(nticks * -1);
		backLeft.setVelocity(nticks * -1);
		backRight.setVelocity(nticks);
		frontLeft.setVelocity(nticks);

	}
  public void strafeleftVelocity(double nticks) {
	frontRight.setVelocity(nticks * -1) ;
	backLeft.setVelocity(nticks * -1) ;
	backRight.setVelocity(nticks) ;
	frontLeft.setVelocity(nticks) ;
  }

  public void runIntakelServo(double position) {
  	// NOTE this takes a value only from 0 to 1
  	intakeServo.setPosition(position);
  }

  // For the other motors, set ticks to 0 to stop the motor
  public void runBucketPower(double power) {
  	bucket.setPower(power) ;
  }

  public void runBucketTurnerVelocity(double ticks) {
  	bucketTurner.setVelocity(ticks) ;
  }

  public long getBucketTurnerPosition() {
  	return bucketTurner.getCurrentPosition() ;
  }

  public void runArmVelocity(double ticks) {
  	armMotor.setVelocity(ticks) ;
  }

  public long getArmPosition() {
  	return armMotor.getCurrentPosition() ;
  }

  public void runCarouselVelocity(double ticks) {
  	carouselTurner.setVelocity(ticks) ;
  }

  public long getCarouselPosition() {
  	return carouselTurner.getCurrentPosition() ;
  }

  // sensors
  public double getLeftFrontDistance() {
  	return distanceLeftFront.getDistance(DistanceUnit.INCH) ;
  }

  public double getLeftBackDistance() {
  	return distanceLeftBack.getDistance(DistanceUnit.INCH) ;
  }

  public double getRightBackDistance() {
  	return distanceRightBack.getDistance(DistanceUnit.INCH) ;
  }

  public double getRightFrontDistance() {
  	return distanceRightFront.getDistance(DistanceUnit.INCH) ;
  }

  public double getFrontDistance() {
  	return distanceFront.getDistance(DistanceUnit.INCH) ;
  }

  public double getFrontColor() {
  	return colorFront.alpha() ;
  }

  public double getIntakeDistance() {
  	return distanceIntake.getDistance(DistanceUnit.INCH) ;
  }

  // Next few are related to the use of IMU
  private void initializeIMU() {
	BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

	parameters.mode				= BNO055IMU.SensorMode.IMU;
	parameters.angleUnit		   = BNO055IMU.AngleUnit.DEGREES;
	parameters.accelUnit		   = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
	parameters.loggingEnabled	  = false;

	// get and initialize the IMU
	// The imu is assumed to be on I2C port
	// and configured to be a sensor of type "AdaFruit IMU" and named "imu"
	imu = hardwareMap.get(BNO055IMU.class, "imu");

	imu.initialize(parameters);

	telemetry.addData("Mode", "IMU calibrating...");
	telemetry.update();

	// make sure the imu gyro is calibrated before continuing.
	while (!isStopRequested() && !imu.isGyroCalibrated())
	{
	  sleep(50);
	  idle();
	}

	telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
	telemetry.update( );
  }


  public void displayOrientationValues(Orientation orient, String name) {
	// ensure that the order is set up correctly as specified in getAngularDistance
	telemetry.addData(name, "Z: " + orient.firstAngle + ", Y: " + orient.secondAngle + ", X: " + orient.thirdAngle);
	telemetry.update() ;
  }

  public void resetAngle() {
	lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

	globalAngle = 0;
  }

  private double getAngle() {

	Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

	double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

	if (deltaAngle < -180)
		deltaAngle += 360;
	else if (deltaAngle > 180)
		deltaAngle -= 360;

	globalAngle += deltaAngle;

	lastAngles = angles;

	return globalAngle;
  }


  private double checkDirection() {

	double angle, gain = .1;

	angle = getAngle();

	if (angle == 0)
		correction = 0;			 // no adjustment.
	else
		correction = -angle;		// reverse sign of angle for correction.

	correction = correction * gain;

	return correction;
  }


  // rotate right based on the degrees
  private void rotateVelocity(int degrees, double ticks) {
	double  leftVelocity, rightVelocity;

	// restart imu movement tracking.
	resetAngle();

	// getAngle() returns + when rotating counter clockwise (left) and - when rotating
	// clockwise (right).

	if (degrees < 0)
	{   // turn right.
		leftVelocity = ticks;
		rightVelocity = -1.0*ticks;
	}
	else if (degrees > 0)
	{   // turn left.
		leftVelocity = -1.0 * ticks;
		rightVelocity = ticks;
	}
	else return;

	// set power to rotate.
	frontLeft.setVelocity(leftVelocity);
	backLeft.setVelocity(leftVelocity);
	frontRight.setVelocity(rightVelocity);
	backRight.setVelocity(rightVelocity);

	// rotate until turn is completed.
	if (degrees < 0)
	{
		// On right turn we have to get off zero first.
		while (getAngle() == 0) {}

		while (getAngle() > degrees) {}
	}
	else	// left turn.
		while (getAngle() < degrees) {}

	// turn the motors off.
	stopDriveMotors();


	// reset angle tracking on new heading.
	resetAngle();
  }

  public void moveForwardIMU(double power) {

	correction = checkDirection();

	backLeft.setPower(power - correction);
	frontLeft.setPower(power - correction);
	backRight.setPower(power + correction);
	frontRight.setPower(power + correction);

  }

  public void moveBackwardIMU(double power) {

	correction = checkDirection();

	backLeft.setPower((power - correction) * -1.0);
	frontLeft.setPower((power - correction) * -1.0);
	backRight.setPower((power + correction) * -1.0);
	frontRight.setPower((power + correction) * -1.0);

  }

  public void strafeRightIMU(double power) {

	correction = checkDirection();

	backLeft.setPower((power - correction));
	frontLeft.setPower((power - correction) * -1.0);
	backRight.setPower((power + correction) * -1.0);
	frontRight.setPower((power + correction));

  }

  public void strafeLeftIMU(double power) {

	correction = checkDirection();

	backLeft.setPower((power + correction) * -1.0);
	frontLeft.setPower((power + correction));
	backRight.setPower((power - correction));
	frontRight.setPower((power - correction) * -1.0);

  }


}

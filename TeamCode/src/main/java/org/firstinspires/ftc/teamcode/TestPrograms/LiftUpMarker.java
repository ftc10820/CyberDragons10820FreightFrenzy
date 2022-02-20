/*
TeleOp:
1. Get infront of Team Marker
2. Press button or somehow changes to Autonomous
Autonomous:
1. Backup a enough to put the arm down
2. Move arm all the way down (Or enough to pick it up)
3. Move forward same amount as Part 1 or enough to connect to the hook thing
4. Move arm up to the top of the shipping area
5. Lower it enough so it touches the pole, and enough so it un-hooks
6. Move Back
7. Switch to TeleOp/End function
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

@Autonomous
public class DropOffMarker extends LinearOpMode {
	private DcMotorEx frontRight;
	private DcMotorEx frontLeft;
	private DcMotorEx backLeft;
	private DcMotorEx backRight;

	//Arm + Other Motors
	private DcMotor bucket;
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

  public void runOpMode() {

	//initialization
	initializeRobot();
	telemetry.addData("Status", "Initialized");
	telemetry.update();

	waitForStart();
	if(opModeIsActive()) {
		long encoderArmValue;
		long encoderBucketValue;
		long encoderFrontLeftValue;
		long encoderFrontRightValue;
		long encoderBackLeftValue;
		long encoderBackRightValue;
		bucketTurner.setTargetPosition(-185);
		armMotor.setTargetPosition(1056);
		bucketTurner.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    bucketTurner.setPower(.5);
    armMotor.setPower(.5);
    while(bucketTurner.isBusy()||armMotor.isBusy()){
      ;
    }
		sleep(5000);
	}
  }



  private void initializeRobot() {
	frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
	frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
	backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
	backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

	bucket = hardwareMap.get(DcMotorEx.class, "Bucket");
	bucketTurner = hardwareMap.get(DcMotorEx.class, "BucketTurner");
	armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");

	armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
	bucketTurner.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

  frontLeft.setDirection(DcMotor.Direction.FORWARD);
  frontRight.setDirection(DcMotor.Direction.REVERSE);
  backLeft.setDirection(DcMotor.Direction.FORWARD);
  backRight.setDirection(DcMotor.Direction.REVERSE);
	bucketTurner.setDirection(DcMotorEx.Direction.REVERSE);
	/*
	armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
	armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

	bucketTurner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
	bucketTurner.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

	*/
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

  public void strafeRightVelocity(double nticks) {
      frontRight.setVelocity(nticks * -1.0) ;
      backLeft.setVelocity(nticks * -1.0) ;
      backRight.setVelocity(nticks) ;
      frontLeft.setVelocity(nticks) ;
  }

  public void runIntakeServo(double position) {
      // NOTE this takes a value only from 0 to 1
      intakeServo.setPosition(position);
  }

  // For the other motors, set ticks to 0 to stop the motor
  public void runBucketPower(double power) {

      bucket.setPower(power);
  }

  public void runBucketTurnerVelocity(double ticks) {

      bucketTurner.setVelocity(ticks);
  }

  public void runArmVelocity(double ticks) {

      armMotor.setVelocity(ticks);
  }

  public void runCarouselVelocity(double ticks) {

      carouselTurner.setVelocity(ticks);
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class DropOffMarker extends LinearOpMode {
  //Motors
	private DcMotorEx frontLeft;
	private DcMotorEx frontRight;
	private DcMotorEx backLeft;
	private DcMotorEx backRight;

  //Arms/Picking Things Up
	private DcMotorEx bucket;
	private DcMotorEx bucketTurner;
	private DcMotorEx armMotor;

  public void runOpMode() {

	//initialization
	initializeRobot();
	telemetry.addData("Status", "Initialized");
	telemetry.update();

	waitForStart();
	//run during op mode
	/*
  1st:
  Arm: 1056
	Bucket: -185
  2nd:
  Arm: 900
  Bucket: -89
  FL: 386
	FR: -396
	BL: 391
	BR: -391
	*/

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
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    bucketTurner.setTargetPosition(185);
		armMotor.setTargetPosition(1056);
    frontLeft.setTargetPosition(386);
    frontRight.setTargetPosition(396);
    backLeft.setTargetPosition(391);
    backRight.setTargetPosition(391);
    bucketTurner.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    bucketTurner.setPower(.5);
    armMotor.setPower(.5);
    frontLeft.setPower(1);
    frontRight.setPower(1);
    backLeft.setPower(1);
    backRight.setPower(1);
    while(bucketTurner.isBusy()||armMotor.isBusy()||frontLeft.isBusy()||frontRight.isBusy()||backLeft.isBusy()||backRight.isBusy()){
      ;
    }
    bucketTurner.setTargetPosition(-98);
    armMotor.setTargetPosition(900);
    bucketTurner.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    bucketTurner.setPower(.5);
    armMotor.setPower(.5);
    while(bucketTurner.isBusy()||armMotor.isBusy()){
      ;
    }
    /*
    frontLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		frontRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		backLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		backRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		frontLeft.setPower(1);
		frontRight.setPower(1);
		backLeft.setPower(1);
		backRight.setPower(1);
    */
    sleep(5000);
		/*
		//Used to see the Encoder Values easy
		while(true){
			encoderArmValue = armMotor.getCurrentPosition();
			encoderBucketValue = bucketTurner.getCurrentPosition();
			encoderFrontLeftValue = frontLeft.getCurrentPosition();
			encoderFrontRightValue = frontRight.getCurrentPosition();
			encoderBackLeftValue = backLeft.getCurrentPosition();
			encoderBackRightValue = backRight.getCurrentPosition();
			telemetry.addData("Arm Encoder Value", + encoderArmValue);
			telemetry.addData("Bucket Encoder Value", + encoderBucketValue);
			telemetry.addData("Front Left Encoder Value", + encoderFrontLeftValue);
			telemetry.addData("Front Right Encoder Value", + encoderFrontRightValue);
			telemetry.addData("Back Left Encoder Value", + encoderBackLeftValue);
			telemetry.addData("Back Right Encoder Value", + encoderBackRightValue);
			telemetry.update();
		}
		*/

		//sleep(2000);
		/*
		encoderStartValue = armMotor.getCurrentPosition();
		armMotor.setVelocity(2000);
		sleep(500);
		encoderEndValue = armMotor.getCurrentPosition();
		armMotor.setVelocity(0);
		telemetry.addData("Encoder Start", + encoderStartValue);
		telemetry.addData("Encoder End", + encoderEndValue);
		telemetry.update();
		*/
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

  private void moveArmUp(double power, int time) {
	armMotor.setPower(power);
	sleep(time);
	stopAll();
  }
  private void moveArmDown(double power, int time) {
	armMotor.setPower(power*-1);
	sleep(time);
	stopAll();
  }
  private void moveForward ( double power, int time){
	frontRight.setPower(power);
	frontLeft.setPower(power);
	backRight.setPower(power);
	backLeft.setPower(power);
	sleep(time);
	stopAll();
  }
  private void moveBack ( double power, int time){
	frontRight.setPower(power * -1);
	frontLeft.setPower(power * -1);
	backRight.setPower(power * -1);
	backLeft.setPower(power * -1);
	sleep(time);
	stopAll();
  }
  private void moveLeft ( double power, int time){
	frontRight.setPower(power);
	backRight.setPower(power * -1);
	frontLeft.setPower(power * -1);
	backLeft.setPower(power);
	sleep(time);
	stopAll();
  }
  private void moveRight ( double power, int time){
	frontRight.setPower(power * -1);
	backRight.setPower(power);
	frontLeft.setPower(power);
	backLeft.setPower(power * -1);
	sleep(time);
	stopAll();
  }
  private void turnLeft ( double power, int time){
	backRight.setPower(power * -1);
	frontRight.setPower(power * -1);
	frontLeft.setPower(power);
	backLeft.setPower(power);
	sleep(time);
  }
  private void turnRight ( double power, int time){
	frontRight.setPower(power);
	backRight.setPower(power);
	frontLeft.setPower(power * -1);
	backLeft.setPower(power * -1);
	sleep(time);
	stopAll();
  }
  private void stopAll () {
	frontRight.setPower(0);
	frontLeft.setPower(0);
	backRight.setPower(0);
	backLeft.setPower(0);
	armMotor.setPower(0);
  }
}
/*
Get Encoder Values
while(true){
  _ = _.getCurrentPosition();
  telemetry.addData("_ Value", _);
  telemetry.update();
}
*/

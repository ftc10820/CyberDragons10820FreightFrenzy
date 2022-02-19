package org.firstinspires.ftc.teamcode.TestPrograms;

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
public class PickUpMarker extends LinearOpMode {
  //Motors
  private DcMotorEx frontRight;
  private DcMotorEx frontLeft;
  private DcMotorEx backLeft;
  private DcMotorEx backRight;

  //Arms/Picking Things Up
  private DcMotorEx bucket;
  private DcMotorEx bucketTurner;
  private DcMotorEx armMotor;

  //Sensors
  private DistanceSensor distanceLeftFront;
  private DistanceSensor distanceLeftBack;
  private DistanceSensor distanceRightFront;
  private DistanceSensor distanceRightBack;
  //private DistanceSensor distanceFront;
  private DistanceSensor distanceCarousel;
  private DistanceSensor distanceIntake;
  private ColorSensor colorFront;

  public void runOpMode() {

	//initialization
	initializeRobot();
	telemetry.addData("Status", "Initialized");
	telemetry.update();

	waitForStart();
	//run during op mode
	if(opModeIsActive()) {
		long encoderArmValue;
		long encoderBucketValue;
		bucketTurner.setTargetPosition(15);
		armMotor.setTargetPosition(230);
		bucketTurner.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
		bucketTurner.setPower(1);
		armMotor.setPower(1);
		while(bucketTurner.isBusy()||armMotor.isBusy()){
		  ;
		}
			encoderArmValue = armMotor.getCurrentPosition();
			encoderBucketValue = bucketTurner.getCurrentPosition();
			telemetry.addData("Arm Encoder Value", + encoderArmValue);
			telemetry.addData("Bucket Encoder Value", + encoderBucketValue);
			telemetry.update();
		sleep(2000);
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

	distanceLeftFront = hardwareMap.get(DistanceSensor.class, "distanceLeftFront");
	distanceLeftBack = hardwareMap.get(DistanceSensor.class, "distanceLeftBack");
	distanceRightFront = hardwareMap.get(DistanceSensor.class, "distanceRightFront");
	distanceRightBack = hardwareMap.get(DistanceSensor.class, "distanceRightBack");
	//distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
	distanceCarousel = hardwareMap.get(DistanceSensor.class, "distanceCarousel");
	distanceIntake = hardwareMap.get(DistanceSensor.class, "distanceIntake");
	colorFront = hardwareMap.get(ColorSensor.class, "colorFront");

	armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
	bucketTurner.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

	bucketTurner.setDirection(DcMotorEx.Direction.REVERSE);
	bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
	armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

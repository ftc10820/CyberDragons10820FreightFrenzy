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

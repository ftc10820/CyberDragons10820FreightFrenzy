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

    waitForStart();
    //run during op mode
    while (opModeIsActive()) {
      moveArmUp(1, 500);
      /*
      moveBack(.5, 500);
      moveArmDown(1, 500);
      moveForward(.5, 500);
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
  }

  private void moveArmUp(double power, int time) {
    armMotor.setPower(power);
    sleep(time);
    armMotor.setPower(0);
  }
  private void moveArmDown(double power, int time) {
    armMotor.setPower(power*-1);
    sleep(time);
    armMotor.setPower(0);
  }
  private void moveForward ( double power, int time){
    frontRight.setPower(power);
    frontLeft.setPower(power);
    backRight.setPower(power);
    backLeft.setPower(power);
    sleep(time);
    stopWheel();
  }
  private void moveBack ( double power, int time){
    frontRight.setPower(power * -1);
    frontLeft.setPower(power * -1);
    backRight.setPower(power * -1);
    backLeft.setPower(power * -1);
    sleep(time);
    stopWheel();
  }
  private void moveLeft ( double power, int time){
    frontRight.setPower(power);
    backRight.setPower(power * -1);
    frontLeft.setPower(power * -1);
    backLeft.setPower(power);
    sleep(time);
    stopWheel();
  }
  private void moveRight ( double power, int time){
    frontRight.setPower(power * -1);
    backRight.setPower(power);
    frontLeft.setPower(power);
    backLeft.setPower(power * -1);
    sleep(time);
    stopWheel();
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
    stopWheel();
  }
  private void stopWheel () {
    frontRight.setPower(0);
    frontLeft.setPower(0);
    backRight.setPower(0);
    backLeft.setPower(0);
  }
}

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

    waitForStart();
    //run during op mode
    while (opModeIsActive()) {
      moveArmUp(1, 1000);
      /*
      moveArmDown(1, 0);
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

    armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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

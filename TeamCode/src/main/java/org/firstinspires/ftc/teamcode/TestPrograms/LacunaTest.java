package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class LacunaTest extends LinearOpMode {

    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private DcMotorEx bucket;
    private DcMotorEx bucketTurner;
    private DcMotorEx armMotor;

    private DistanceSensor distanceLeftFront;
    private DistanceSensor distanceLeftBack;
    private DistanceSensor distanceRightFront;
    private DistanceSensor distanceRightBack;
    private DistanceSensor distanceFront;
    private DistanceSensor distanceIntake;
    private ColorSensor colorFront;

    //private Servo intakeServo;


    public void runOpMode() {

        //initialization
        initializeRobot();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //wait for game to start
        waitForStart();

        //run during op mode
        if (opModeIsActive()) {


            pickupFreight(1);
            moveArm(1, 1000);
            sleep(100);
            turnLeft(1, 700);
            sleep(100);
            moveRight(1, 200);
            sleep(100);
            moveForward(1,200);
            /*turnRight(1,150);
            //dropFreight(1, 1000);
            turnLeft(1, 150);
            moveRight(1,200);
            moveBack(1,200);
            turnRight(1,700);
            moveLeft(1,200);
            */



        /*
        moveForward(1, 5000);
        moveBack(1, 5000);
        moveRight(1, 5000);
        moveLeft(1, 5000);
        turnRight(1, 5000);
        turnLeft(1,5000);
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
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        distanceIntake = hardwareMap.get(DistanceSensor.class, "distanceIntake");
        colorFront = hardwareMap.get(ColorSensor.class, "colorFront");

        // configuring the drive motors
        frontLeft.setDirection(DcMotor.Direction.FORWARD) ;
        frontRight.setDirection(DcMotor.Direction.REVERSE) ;
        backLeft.setDirection(DcMotor.Direction.FORWARD) ;
        backRight.setDirection(DcMotor.Direction.REVERSE) ;

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // configuring the other motors
        armMotor.setDirection(DcMotor.Direction.FORWARD) ;
        bucketTurner.setDirection(DcMotor.Direction.REVERSE) ;
        bucket.setDirection(DcMotor.Direction.FORWARD) ;

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucketTurner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    private void moveForward(double power, int time) {
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
        sleep(time);
        stopWheel();
    }

    private void moveBack(double power, int time) {
        frontRight.setPower(power * -1);
        frontLeft.setPower(power * -1);
        backRight.setPower(power * -1);
        backLeft.setPower(power * -1);
        sleep(time);
        stopWheel();
    }

    private void moveLeft(double power, int time) {
        frontRight.setPower(power);
        backRight.setPower(power * -1);
        frontLeft.setPower(power * -1);
        backLeft.setPower(power);
        sleep(time);
        stopWheel();
    }

    private void moveRight(double power, int time) {
        frontRight.setPower(power * -1);
        backRight.setPower(power);
        frontLeft.setPower(power);
        backLeft.setPower(power * -1);
        sleep(time);
        stopWheel();
    }

    private void stopWheel() {
        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }

    private void turnRight(double power, int time) {
        frontRight.setPower(power);
        backRight.setPower(power);
        frontLeft.setPower(power * -1);
        backLeft.setPower(power * -1);
        sleep(time);
        stopWheel();
    }
    private void turnLeft ( double power, int time) {
        frontRight.setPower(power * -1);
        backRight.setPower(power * -1);
        frontLeft.setPower(power);
        backLeft.setPower(power);
        sleep(time);
    }
    private void pickupFreight (double power) {
        bucket.setPower(power);
        while(distanceIntake.getDistance(DistanceUnit.INCH) > 2.0) {
        telemetry.addData("distanceIntake: ", "" + distanceIntake.getDistance(DistanceUnit.INCH));
        telemetry.update();
        }
        bucket.setPower(0);
    }

    private void moveArm (double power, int time){
        armMotor.setPower(power);
    }

    private void dropFreight (double power, int time){
        bucketTurner.setPower(power*-1);
        sleep(200);
        //intakeServo();
    }

}

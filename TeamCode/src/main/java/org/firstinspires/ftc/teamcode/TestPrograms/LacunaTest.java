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

    @Override
    public void runOpMode() throws InterruptedException {

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

    // add functions here
    private void initializeRobot() {

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
        //add imu later

        // sets the motors to use encoders
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bucket.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bucketTurner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        carouselTurner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //reset encoders
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bucket.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
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

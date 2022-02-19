/*
 make a copy of this file and use it as a template to start your programming;
 initialization and all hardware components are done, just start creating
 your opmode, whether it be autonomous or teleop
 */

package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

//uncomment the mode, whether it be teleop or autonomous
//@TeleOp
//@Autonomous
public class CyberDragonsOpModeTemplate extends LinearOpMode {

    //Motors
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
        bucket = hardwareMap.get(DcMotor.class, "Bucket");
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

    public void armWithBucket(int encValArm, double pvalArm, int encValBuc, double pvalBuc) {


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

}


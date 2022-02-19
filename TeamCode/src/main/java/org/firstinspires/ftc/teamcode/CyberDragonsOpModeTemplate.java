/*
 make a copy of this file and use it as a template to start your programming;
 initialization and all hardware components are done, just start creating
 your opmode, whether it be autonomous or teleop
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

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
    private DcMotorEx bucket;
    private DcMotorEx bucketTurner;
    private DcMotorEx armMotor;

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
        while (opModeIsActive()) {

            //code here to run during the opmode, note that it is a while loop


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

        // initializing sensors
        distanceLeftFront = hardwareMap.get(DistanceSensor.class, "distanceLeftFront");
        distanceLeftBack = hardwareMap.get(DistanceSensor.class, "distanceLeftBack");
        distanceRightFront = hardwareMap.get(DistanceSensor.class, "distanceRightFront");
        distanceRightBack = hardwareMap.get(DistanceSensor.class, "distanceRightBack");
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        distanceIntake = hardwareMap.get(DistanceSensor.class, "distanceIntake");
        colorFront = hardwareMap.get(ColorSensor.class, "colorFront");

        // sets the motors to use encoders (uncomment if using encoders)
        /*

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bucket.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bucketTurner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        */

        // sets the motors to brake when there is no power applied (motor tries to actively maintain position)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucketTurner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

}


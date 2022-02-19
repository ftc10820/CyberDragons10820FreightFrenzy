package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
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
public class ShippingHubAutomationLevels extends LinearOpMode {

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
    //private DistanceSensor distanceFront;
    private DistanceSensor distanceCarousel;
    private DistanceSensor distanceIntake;
    private ColorSensor colorFront;


    public enum LiftStates {

        LIFT_START,
        LIFT_LEVELONE,
        LIFT_LEVELTWO,
        LIFT_LEVELTHREE;
    }


    LiftStates liftstates = LiftStates.LIFT_START;

    int LINEAR_SLIDE_HIGH = -150;
    int LINEAR_SLIDE_LOW = -15;

    int BUCKET_TURNER_HIGH = -30;
    int BUCKET_TURNER_LOW = -20;

    double BUCKET_IDLE = 0;
    double BUCKET_RELEASE = 1;

    double RELEASE_TIME = 5000;

    ElapsedTime releaseTimer = new ElapsedTime();

    double speed = 1200; //arbitrary number; static to allow for analyzing how PID performs through multiple speeds in dashboard

    PIDCoefficients pidCoeffs = new PIDCoefficients(0, 0, 0); //PID coefficients that need to be tuned probably through FTC dashboard
    PIDCoefficients pidGains = new PIDCoefficients(0, 0, 0); //PID gains which we will define later in the process

    ElapsedTime PIDTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS); //timer


    public void runOpMode() {

        //initialization
        //initializeRobot();

        waitForStart();

        //run during op mode
        while (opModeIsActive()) {

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





            /*
            public void loop() {
                switch (liftstates) {
                    case LiftStates.LIFT_START:
                        // Waiting for some input
                        if (gamepad1.x) {
                            // x is pressed, start extending
                            armMotor.setPosition(ARM_LOW);
                            liftState = LiftState.LIFT_EXTEND;
                        }
                        break;

                }
            }
            */
        }
    }
}
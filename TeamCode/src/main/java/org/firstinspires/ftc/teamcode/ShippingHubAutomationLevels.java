package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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


    public void runOpMode() {

        //initialization
        initializeRobot();

        waitForStart();

        //run during op mode
        while (opModeIsActive()) {

            public enum LiftStates {
                LIFT_START,
                LIFT_LEVELONE,
                LIFT_LEVELTWO,
                LIFT_LEVELTHREE

                LiftStates liftstates = LiftStates LIFT_START;

            }

        }
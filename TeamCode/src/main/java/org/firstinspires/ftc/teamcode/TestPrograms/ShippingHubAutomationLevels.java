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
public class ShippingHubAutomationLevels extends CyberDragonsOpModeTemplate {

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

    public void dropFreightInLevel(double level) {

        if (level == 1) {

            armWithBucket(0, 0.5, 0, 0.5);

        } else if (level == 2) {

            armWithBucket(0, 0.5, 0, 0.5);

        } else if (level == 3) {

            armWithBucket(0, 0.5, 0, 0.5);

        }
    }

}
package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous
@Disabled
public class ShippingHubAutomationLevels extends CyberDragonsOpModeTemplate {


    public void runOpMode() {

        //initialization
        initializeRobot();

        waitForStart() ;

        //run during op mode
        if (opModeIsActive()) {
            telemetry.addData("Status", "Running..") ;
            telemetry.update() ;

            // what did the camera see??
            // the real work
            //dropFreightInLevel(3);
            moveForwardVelocity(2000) ;
            sleep(1000) ;
            stopDriveMotors();

        }
    }

    public void dropFreightInLevel(double level) {

        if (level == 1) {

            // move to the correct location
            if (getFrontDistance() > 12.0) {
                moveForwardVelocity(2000) ;
                while (getFrontDistance() >= 12.0) {
                    ;
                }
                stopDriveMotors();
            } else if (getFrontDistance() < 11.5) { // more important
                moveBackwardVelocity(2000) ;
                while (getFrontDistance() <= 11.5) {
                    ;
                }
                stopDriveMotors();
            }
            moveArmWithBucket(0, 0.5, -350, 0.5);

        } else if (level == 2) {

            // move to the correct location
            if (getFrontDistance() > 10.0) {
                moveForwardVelocity(2000) ;
                while (getFrontDistance() >= 10.0) {
                    ;
                }
                stopDriveMotors();
            } else if (getFrontDistance() < 9.5) {
                moveBackwardVelocity(2000) ;
                while (getFrontDistance() <= 9.5) {
                    ;
                }
                stopDriveMotors();
            }
            moveArmWithBucket(660, 0.5, -1000, 0.5);

        } else if (level == 3) {

            // move to the correct location
            if (getFrontDistance() > 10.0) {
                moveForwardVelocity(2000) ;
                while (getFrontDistance() >= 10.0) {
                    ;
                }
                stopDriveMotors();
            } else if (getFrontDistance() < 9.5) {
                moveBackwardVelocity(2000) ;
                while (getFrontDistance() <= 9.5) {
                    ;
                }
                stopDriveMotors();
            }
            moveArmWithBucket(950, 0.5, -1150, 0.5);

            moveForwardVelocity(2000) ;
            while (getFrontDistance() > 3.0) {
                ;
            }
            stopDriveMotors();

        } else {
            telemetry.addData("Error", "Invalid level") ;
            telemetry.update() ;
            return ;
        }

        // eject the freight
        runBucketPower(1) ;
        sleep(5000) ;
        runBucketPower(0) ;
    }

}
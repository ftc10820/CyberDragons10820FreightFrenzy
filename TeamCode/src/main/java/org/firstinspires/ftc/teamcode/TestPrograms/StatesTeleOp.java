package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;
import org.checkerframework.checker.units.qual.Current;

public class StatesTeleOp extends OpMode  {

    CyberDragonsOpModeTemplate cyberDragonsOpModeTemplate = new CyberDragonsOpModeTemplate();
    ShippingHubAutomationLevels shippingHubAutomationLevels = new ShippingHubAutomationLevels();

    CyberDragons drive = new CyberDragons(hardwareMap);

    /*
    integrations necessary
    - automation for different levels
    - rift + lacuna automations
    - carousel automation
    - capping
     */

    private enum CurrentMode {

        DRIVER_CONTROL,
        LEVEL_3,
        LACUNA,
        PICKUP_MARKER,
        CAPPING

    }

    CurrentMode currentMode = CurrentMode.DRIVER_CONTROL;

    @Override
    public void init() {

        cyberDragonsOpModeTemplate.initializeRobot();

    }

    @Override
    public void loop() {

        telemetry.addData("Current Stage", currentMode);
        telemetry.update();

        switch (currentMode) {
            case LEVEL_3:

                //rift automation
                //shippingHubAutomationLevels.dropFreightInLevel(3);

                break;

            case LACUNA:

                //lacuna automation

                break;

            case CAROUSEL:

                //carousel automation


                break;

            case CAPPING:

                // capping automation

                break;

            case DRIVER_CONTROL:

                //normal driver
                double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                double rx = -gamepad1.right_stick_x ;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio, but only when
                // at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double frontLeftPower = (y + x + rx) / denominator;
                double backLeftPower = (y - x + rx) / denominator;
                double frontRightPower = (y - x - rx) / denominator;
                double backRightPower = (y + x - rx) / denominator;

                break;
            default:
                currentMode = CurrentMode.DRIVER_CONTROL;
        }

        // temporary fail-safe is dpad up
        if ((gamepad1.right_bumper || gamepad1.left_bumper) && currentMode != CurrentMode.DRIVER_CONTROL) {

            currentMode = CurrentMode.DRIVER_CONTROL;

        }

    }

    public void setThrottle(double frontLeftVelocity, double frontRightVelocity, double backLeftVelocity, double backRightVelocity) {
        // Constrain throttle values to between -1.0 to 1.0
        if (frontLeftVelocity > 1.0) {

            frontLeftVelocity = 1.0;

        } else if (frontLeftVelocity < -1.0) {

            frontLeftVelocity = -1.0;

        }

        if (frontRightVelocity > 1.0) {

            frontRightVelocity = 1.0;

        } else if (frontRightVelocity < -1.0) {

            frontRightVelocity = -1.0;

        }

        if (backLeftVelocity > 1.0) {

            backLeftVelocity = 1.0;

        } else if (backLeftVelocity < -1.0) {

            backLeftVelocity = -1.0;

        }

        if (backRightVelocity > 1.0) {

            backRightVelocity = 1.0;

        } else if (backRightVelocity < -1.0) {

            backRightVelocity = -1.0;

        }

        // Set velocities
        drive.frontLeft.setVelocity(frontLeftVelocity * PID_MecanumDrive.MAX_TICKS_PER_SECOND);
        drive.frontRight.setVelocity(frontRightVelocity * PID_MecanumDrive.MAX_TICKS_PER_SECOND);
        drive.backLeft.setVelocity(backLeftVelocity * PID_MecanumDrive.MAX_TICKS_PER_SECOND);
        drive.backRight.setVelocity(backRightVelocity * PID_MecanumDrive.MAX_TICKS_PER_SECOND);
    }

}

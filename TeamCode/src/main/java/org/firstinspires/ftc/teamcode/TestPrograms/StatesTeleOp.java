package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.checkerframework.checker.units.qual.C;
import org.checkerframework.checker.units.qual.Current;

public class StatesTeleOp extends OpMode  {

    CyberDragonsOpModeTemplate cyberDragonsOpModeTemplate = new CyberDragonsOpModeTemplate();
    ShippingHubAutomationLevels shippingHubAutomationLevels = new ShippingHubAutomationLevels();

    /*
    integrations necessary
    - automation for different levels
    - rift + lacuna automations
    - carousel automation
    - capping
     */

    private enum CurrentMode {

        DRIVER_CONTROL,
        RIFT,
        LACUNA,
        CAROUSEL,
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
            case RIFT:

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
        if (gamepad1.dpad_up && currentMode != CurrentMode.DRIVER_CONTROL) {

            currentMode = CurrentMode.DRIVER_CONTROL;

        }

    }
}

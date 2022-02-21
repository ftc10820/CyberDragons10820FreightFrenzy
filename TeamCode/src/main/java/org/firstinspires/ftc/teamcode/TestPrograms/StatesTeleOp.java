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

    enum CurrentMode {

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

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


    public void runOpMode() {

        //initialization
        initializeRobot();

        waitForStart() ;

        //run during op mode
        if (opModeIsActive()) {

            dropFreightInLevel(3);


        }
    }

    public void dropFreightInLevel(double level) {

        if (level == 1) {

            armWithBucket(0, 0.5, -350, 0.5);

        } else if (level == 2) {

            armWithBucket(660, 0.5, -1000, 0.5);

        } else if (level == 3) {

            armWithBucket(950, 0.5, -1150, 0.5);

        }
    }

}
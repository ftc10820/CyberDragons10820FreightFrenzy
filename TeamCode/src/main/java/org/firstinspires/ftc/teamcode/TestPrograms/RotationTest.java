package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
//@Disabled
public class RotationTest extends LinearOpMode{

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private DcMotorEx intakeMotor ;

    DistanceSensor dLF, dLB, dRF, dRB ;
    DistanceSensor dF, dIn, dCar ;
    ColorSensor cF ;

    @Override
    public void runOpMode() {

        int start_enc_val, end_enc_val ;

        initializeRobot();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {

            telemetry.addData("Status", "Running");
            telemetry.update();

            testSensors();

            /*
            frontLeft.setPower(-0.5) ;
            frontRight.setPower(0.5) ;
            backLeft.setPower(0.5) ;
            backRight.setPower(-0.5) ;

            sleep(5000) ;
            frontLeft.setPower(0.0) ;
            frontRight.setPower(0.0) ;
            backLeft.setPower(0.0) ;
            backRight.setPower(0.0) ;
            sleep(1000) ;
           */
        }
    }


    private void initializeRobot() {

        frontLeft = hardwareMap.get(DcMotorEx.class,"FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class,"FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class,"BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class,"BackRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD) ;
        frontRight.setDirection(DcMotor.Direction.REVERSE) ;
        backLeft.setDirection(DcMotor.Direction.FORWARD) ;
        backRight.setDirection(DcMotor.Direction.REVERSE) ;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;

        intakeMotor = hardwareMap.get(DcMotorEx.class,"FrontLeft");;

        // hardware map for sensors
        dLF = hardwareMap.get(DistanceSensor.class, "distanceLeftFront");
        dLB = hardwareMap.get(DistanceSensor.class, "distanceLeftBack");
        dRF = hardwareMap.get(DistanceSensor.class, "distanceRightFront");
        dRB = hardwareMap.get(DistanceSensor.class, "distanceRightBack");
        //dF = hardwareMap.get(DistanceSensor.class, "distanceFront");
        dIn = hardwareMap.get(DistanceSensor.class, "distanceIntake");
        dCar = hardwareMap.get(DistanceSensor.class, "distanceCarousel");

        cF = hardwareMap.get(ColorSensor.class, "colorFront");

    }

    private void testSensors() {
        double lfd, lbd, rfd, rbd;
        double fd, ind, card ;
        double fc ;

        long cur_time = System.currentTimeMillis() ;
        while ((System.currentTimeMillis() - cur_time) < 20000) {
            lfd = dLF.getDistance(DistanceUnit.INCH) ;
            lbd = dLB.getDistance(DistanceUnit.INCH) ;
            rfd = dRF.getDistance(DistanceUnit.INCH) ;
            rbd = dRB.getDistance(DistanceUnit.INCH) ;
            //fd = dF.getDistance(DistanceUnit.INCH) ;
            ind = dIn.getDistance(DistanceUnit.INCH) ;
            card = dCar.getDistance(DistanceUnit.INCH) ;
            fc = cF.alpha() ;

            telemetry.addData("Sensors", "LF: " + lfd + ", LB: " + lbd);
            telemetry.addData("Sensors", "RF: " + rfd + ", RB: " + rbd);
            telemetry.addData("Sensors", "in: " + ind + ", Crsl: " + card);
            telemetry.addData("Sensors", "col: " + fc );

            telemetry.update();

        }

    }
}


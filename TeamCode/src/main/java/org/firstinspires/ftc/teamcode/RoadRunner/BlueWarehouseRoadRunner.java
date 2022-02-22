package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.google.ar.core.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TestPrograms.Carousel;
import org.firstinspires.ftc.teamcode.TestPrograms.CyberDragonsOpModeTemplate;
import org.firstinspires.ftc.teamcode.TestPrograms.ShippingHubAutomationLevels;


@Autonomous
public class BlueWarehouseRoadRunner extends LinearOpMode {

    private DcMotorEx carouselTurner;
    private DcMotorEx armMotor;
    private DcMotorEx bucketTurner;
    private DcMotor bucket;

    ElapsedTime carouselTimer = new ElapsedTime();
    ElapsedTime releaseTimer = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(10, 70, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        Trajectory shippingHub = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-15, 50), Math.toRadians(-90))
                .build();

        Trajectory intakeFreight = drive.trajectoryBuilder(shippingHub.end().plus(new Pose2d(0,0,Math.toRadians(90))))
                .splineTo(new Vector2d(30, 80), Math.toRadians(0))
                .lineTo(new Vector2d(50,80))
                .build();

        Trajectory placeFreight = drive.trajectoryBuilder(intakeFreight.end())
                .lineTo(new Vector2d(30,80))
                .splineTo(new Vector2d(-15,50), Math.toRadians(90))
                .build();

        /*
        Trajectory placeFreightOnHub = drive.trajectoryBuilder(placeFreight.end().plus(new Pose2d(0,0, Math.toRadians(-90))))
                .lineTo(new Vector2d(-15,50))
                .build();
        */

        Trajectory parkWarehouse = drive.trajectoryBuilder(placeFreight.end().plus(new Pose2d(0,0,Math.toRadians(90))))
                .splineTo(new Vector2d(30, 80), Math.toRadians(0))
                .lineTo(new Vector2d(50,80))
                .build();

        initializeMotors();

        waitForStart();

        if (opModeIsActive()) {

            // do carousel
            drive.followTrajectory(shippingHub);
            sleep(500);

            drive.turn(Math.toRadians(90));
            sleep(500);

            drive.followTrajectory(intakeFreight);
            sleep(500);


            drive.followTrajectory(placeFreight);
            sleep(500);

            /*
            drive.turn(Math.toRadians(-90));
            sleep(500);


            drive.followTrajectory(placeFreightOnHub);
            sleep(500);

            */

            drive.turn(Math.toRadians(90));
            sleep(500);


            drive.followTrajectory(parkWarehouse);
            sleep(500);


        }
    }

    private void initializeMotors() {

        carouselTurner = hardwareMap.get(DcMotorEx.class, "CarouselTurner");
        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        bucketTurner = hardwareMap.get(DcMotorEx.class, "BucketTurner");
        bucket = hardwareMap.get(DcMotor.class, "Bucket");

        carouselTurner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bucketTurner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        carouselTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        carouselTurner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucketTurner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bucketTurner.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void deliverDuck() {

        carouselTimer.reset();

        carouselTurner.setVelocity(1000);
        while (carouselTimer.milliseconds() < 500) {

        }

        carouselTurner.setVelocity(1250);
        while (carouselTimer.milliseconds() < 1000) {

        }

        carouselTurner.setVelocity(1750);
        while (carouselTimer.milliseconds() < 2000) {

        }

        carouselTurner.setVelocity(0);

    }

    public void dropFreightInLevel(int level) {

        if (level == 1) {

            moveArmToEncoderVal(150, 0.5);
            sleep(1000);
            moveBucketToEncoderVal(-650, 0.5) ;
            sleep(1000) ;

        } else if (level == 2) {
            moveArmToEncoderVal(650, 0.5) ;
            sleep(1000) ;
            moveBucketToEncoderVal(-1050, 0.5) ;
            sleep(1000) ;

        } else if (level == 3) {
            moveArmToEncoderVal(950, 0.5) ;
            sleep(1000) ;
            moveBucketToEncoderVal(-1150, 0.5) ;
            sleep(1000) ;

        } else {
            telemetry.addData("Error", "Invalid level") ;
            telemetry.update() ;
            return ;
        }

        releaseTimer.reset();

        bucket.setPower(-1);
        while (releaseTimer.milliseconds() < 1000) {

        }

        bucket.setPower(0);

    }

    public void moveArmToEncoderVal(int encValArm, double pval) {

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setTargetPosition(encValArm);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(pval);

        while (armMotor.isBusy()) {


        }

    }

    public void moveBucketToEncoderVal(int encValBuc, double pval) {

        bucketTurner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        bucketTurner.setTargetPosition(encValBuc);

        bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        bucketTurner.setPower(pval);

        while (bucketTurner.isBusy()) {


        }

    }

    public void moveArmWithBucket(int encValArm, double pvalArm, int encValBuc, double pvalBuc) {


        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bucketTurner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setTargetPosition(encValArm);
        bucketTurner.setTargetPosition(encValBuc);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(pvalArm);
        bucketTurner.setPower(pvalBuc);

        while (armMotor.isBusy() || bucketTurner.isBusy()) {


        }

    }

}
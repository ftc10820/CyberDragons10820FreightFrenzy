package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
public class RedCarouselRoadRunner extends LinearOpMode {

    private DcMotorEx carouselTurner;
    private DcMotorEx armMotor;
    private DcMotorEx bucketTurner;
    private DcMotor bucket;

    ElapsedTime carouselTimer = new ElapsedTime();

    ElapsedTime releaseTimer = new ElapsedTime();

    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -65, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory carouselTurner = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-65, -65))
                .build();

        Trajectory shippingHub = drive.trajectoryBuilder(carouselTurner.end())
                .splineTo(new Vector2d(-15, -50), Math.toRadians(90))
                .build();

        Trajectory parkWarehouse = drive.trajectoryBuilder(shippingHub.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .lineTo(new Vector2d(60, -50))
                .build();

        initializeMotors();

        waitForStart();

        if (opModeIsActive()) {

            // do carousel
            drive.followTrajectory(carouselTurner);
            sleep(100);

            deliverDuck();

            sleep(100);
            /*
            carouselTurner.setVelocity(200);
            sleep(500);

            carouselTurner.setVelocity(500);
            sleep(500);

            carouselTurner.setVelocity(1000);
            sleep(2000);

            carouselTurner.setVelocity(2000);
            sleep(2000);

            carouselTurner.setvelocity(0);
            sleep(20000);
            */

            // place freight
            drive.followTrajectory(shippingHub);
            sleep(100);

            dropFreightInLevel(3);

            sleep(100);

            drive.turn(Math.toRadians(-90));
            sleep(100);

            drive.followTrajectory(parkWarehouse);


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

        carouselTurner.setVelocity(-1000);
        while (carouselTimer.milliseconds() < 500) {

        }

        carouselTurner.setVelocity(-1250);
        while (carouselTimer.milliseconds() < 1000) {

        }

        carouselTurner.setVelocity(-1750);
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
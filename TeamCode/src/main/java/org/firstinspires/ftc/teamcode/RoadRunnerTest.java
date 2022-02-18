package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RoadRunnerTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive drive = new org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, 65, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        Trajectory carouselTurner = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-70, 65))
                .build();

        Trajectory shippingHub = drive.trajectoryBuilder(carouselTurner.end())
                .splineToConstantHeading(new Vector2d(-15, 40), Math.toRadians(0))
                .build();

        Trajectory parkWarehouse = drive.trajectoryBuilder(shippingHub.end())
                .lineTo(new Vector2d(60, 40))
                .build();

        waitForStart();

        if (opModeIsActive()) {

            drive.followTrajectory(carouselTurner);
            // do carousel
            drive.followTrajectory(shippingHub);
            // place freight
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(parkWarehouse);


        }
    }

}
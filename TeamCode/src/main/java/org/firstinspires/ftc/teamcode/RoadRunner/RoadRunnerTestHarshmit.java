package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RoadRunnerTestHarshmit extends LinearOpMode {

    public void runOpMode() throws InterruptedException {



        org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive drive = new org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, 65, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        Trajectory mytrajectory= drive. trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(0,10))
                .build();

        waitForStart();

        if (opModeIsActive()) {

            drive.followTrajectory(mytrajectory);
            // test for Harshmit
            //drive.turn(Math.toRadians(90));
            //drive.followTrajectory(parkWarehouse);


        }
    }

}
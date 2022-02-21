package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TestPrograms.Carousel;
import org.firstinspires.ftc.teamcode.TestPrograms.CyberDragonsOpModeTemplate;
import org.firstinspires.ftc.teamcode.TestPrograms.ShippingHubAutomationLevels;

@Autonomous
public class RoadRunnerTest extends LinearOpMode {

    Carousel carousel = new Carousel();

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    ShippingHubAutomationLevels shippingHubAutomationLevels = new ShippingHubAutomationLevels();

    public void runOpMode() throws InterruptedException {


        Pose2d startPose = new Pose2d(-35, 65, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        Trajectory carouselTurner = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(0, 65))
                .build();

        Trajectory shippingHub = drive.trajectoryBuilder(carouselTurner.end())
                .splineTo(new Vector2d(-60, 60), Math.toRadians(0))
                .build();

        Trajectory parkWarehouse = drive.trajectoryBuilder(shippingHub.end())
                .lineTo(new Vector2d(60, 40))
                .build();

        waitForStart();

        if (opModeIsActive()) {

            // do carousel
            drive.followTrajectory(carouselTurner);
            //carousel.deliverDuck();

            // place freight
            drive.followTrajectory(shippingHub);
            //shippingHubAutomationLevels.dropFreightInLevel(3);
            
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(parkWarehouse);


        }
    }


}
package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TestPrograms.Carousel;
import org.firstinspires.ftc.teamcode.TestPrograms.CyberDragonsOpModeTemplate;
import org.firstinspires.ftc.teamcode.TestPrograms.ShippingHubAutomationLevels;


@Autonomous
public class RoadRunnerTest extends LinearOpMode {
    private DcMotorEx carouselTurner;
    private void initializeCarousel() {
        carouselTurner = hardwareMap.get(DcMotorEx.class, "CarouselTurner");
        carouselTurner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    //Carousel carousel = new Carousel();

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    ShippingHubAutomationLevels shippingHubAutomationLevels = new ShippingHubAutomationLevels();

    public void runOpMode() throws InterruptedException {


        Pose2d startPose = new Pose2d(-35, 65, Math.toRadians(-90));

        drive.setPoseEstimate(startPose);

        Trajectory carouselTurner = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(0, 65))
                .build();

        Trajectory shippingHub = drive.trajectoryBuilder(carouselTurner.end())
                .splineTo(new Vector2d(-50, 50), Math.toRadians(-90))
                .build();

        Trajectory parkWarehouse = drive.trajectoryBuilder(shippingHub.end().plus(new Pose2d(0,0,Math.toRadians(-80))))
                .lineTo(new Vector2d(-130, 50))
                .build();

        initalizeCarousel();

        waitForStart();

        if (opModeIsActive()) {

            // do carousel
            drive.followTrajectory(carouselTurner);
            sleep(10000);
            //carousel.deliverDuck();
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

            // place freight
            drive.followTrajectory(shippingHub);
            //shippingHubAutomationLevels.dropFreightInLevel(3);
            
            drive.turn(Math.toRadians(-80));
            drive.followTrajectory(parkWarehouse);


        }
    }


}
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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.TestPrograms.Carousel;
import org.firstinspires.ftc.teamcode.TestPrograms.CyberDragonsOpModeTemplate;
import org.firstinspires.ftc.teamcode.TestPrograms.ShippingHubAutomationLevels;

import java.util.List;


@Autonomous(name = "Red: Carousel Autonomous", group = "Red")
public class RedCarouselRoadRunner extends LinearOpMode {

    private DcMotorEx carouselTurner;
    private DcMotorEx armMotor;
    private DcMotorEx bucketTurner;
    private DcMotor bucket;

    ElapsedTime carouselTimer = new ElapsedTime();

    ElapsedTime releaseTimer = new ElapsedTime();

    // object detection variables
    private static final String TFOD_MODEL_ASSET = "10820model.tflite";
    private static final String[] LABELS = new String[] { "10820marker" };
    private static final String VUFORIA_KEY = "Af2A/t3/////AAABmSsJTGsI6Ebgr6cIo4YGqmCBxd+lRenqxeIeJ3TQXcQgRlvrzKhb44K7xnbfJnHjD6eLQaFnpZZEa1Vz1PRYMNj3xCEhYZU7hAYQwyu1KBga3Lo0vEPXPSZW1o8DrM2C6IhYYGifzayZFNwZw5HtnPbyZvJfG4w6TX4EO8F0VSnZt87QtBW27nh5vSgRLN1XdzrVzm8h1ScZrPsIpSKJWVmNCWqOOeibloKfoZbhZ5A8vFz0I3nvMdi/v54DwcmS7GS/hryCgjhy4n9EhD1SnJ5325jnoyi4Fa5a/pibxPmAi8kU7ioHucmQRgv3yQHh17emqait9QNS4jTu6xyM6eeoVADsXTG4f7KK6nlZZjat";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    int objectsRecognized = 0;
    int level = 0;
    int xPosMarker = 150;

    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-40, -65, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory carouselTurner = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(-70, -75))
                .build();

        Trajectory shippingHub = drive.trajectoryBuilder(carouselTurner.end())
                .splineTo(new Vector2d(-30, -55), Math.toRadians(90))
                .build();

        Trajectory parkWarehouse = drive.trajectoryBuilder(shippingHub.end().plus(new Pose2d(0,0,Math.toRadians(-90))))
                .lineTo(new Vector2d(50, -55))
                .build();

        initializeMotors();

        waitForStart();


        if (opModeIsActive()) {


            objectDetection();
            telemetry.addData("Level", level);
            telemetry.update();

            // do carousel
            drive.followTrajectory(carouselTurner);
            sleep(100);

            deliverDuck();

            sleep(100);

            drive.followTrajectory(shippingHub);
            sleep(100);

            dropFreightInLevel(level);

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


        initVuforia();
        initTfod();

        if (tfod != null) {

            tfod.activate();
            tfod.setZoom(1.0, 2.0);

        }

    }

    public void deliverDuck() {

        carouselTurner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        carouselTimer.reset();

        carouselTurner.setPower(0.25);
        while (carouselTimer.milliseconds() < 500) {

        }

        carouselTurner.setPower(0.5);
        while (carouselTimer.milliseconds() < 1000) {

        }

        carouselTurner.setPower(0.75);
        while (carouselTimer.milliseconds() < 2750) {

        }

        carouselTurner.setPower(0);

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

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Camera");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    private void objectDetection() {
        float leftVal = 0.0F;
        if (opModeIsActive())
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", Integer.valueOf(updatedRecognitions.size()));
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", new java.lang.Object[] { Integer.valueOf(i) }), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", new java.lang.Object[] { Integer.valueOf(i) }), "%.03f , %.03f", new java.lang.Object[] { Float.valueOf(recognition.getLeft()), Float.valueOf(recognition.getTop()) });
                        telemetry.addData(String.format("  right,bottom (%d)", new java.lang.Object[] { Integer.valueOf(i) }), "%.03f , %.03f", new java.lang.Object[] { Float.valueOf(recognition.getRight()), Float.valueOf(recognition.getBottom()) });
                        i++;
                        objectsRecognized++;
                        leftVal = recognition.getLeft();
                    }
                    if (leftVal <= xPosMarker && objectsRecognized == 1) {
                        level = 1;
                        telemetry.addData("Level", Integer.valueOf(level));
                        telemetry.update();
                    } else if (leftVal >= xPosMarker && objectsRecognized == 1) {
                        level = 2;
                        telemetry.addData("Level", Integer.valueOf(level));
                        telemetry.update();
                    } else if (objectsRecognized == 0) {
                        level = 3;
                        telemetry.addData("Level", Integer.valueOf(level));
                        telemetry.update();
                    }
                    telemetry.update();
                }
            }
    }

}
package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous
@Disabled
public class BlueWarehouseAutonomous extends LinearOpMode {
    private DcMotor frontLeft;

    private DcMotor frontRight;

    private DcMotor backLeft;

    private DcMotor backRight;

    private DcMotor bucket;

    private DcMotor bucketTurner;

    private DcMotor linearSlide;

    private DcMotor carouselTurner;

    private Servo Marker;

    private DistanceSensor FrontRightDistance;

    private DistanceSensor FrontLeftDistance;

    private DistanceSensor BackRightDistance;

    private DistanceSensor BackLeftDistance;

    private DistanceSensor CarouselDistance;

    private DistanceSensor FrontDistance;

    private ColorSensor FrontColor;

    private ColorSensor BackColor;

    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/10820model.tflite";

    private static final String[] LABELS = new String[] { "10820marker" };

    private static final String VUFORIA_KEY = "Af2A/t3/////AAABmSsJTGsI6Ebgr6cIo4YGqmCBxd+lRenqxeIeJ3TQXcQgRlvrzKhb44K7xnbfJnHjD6eLQaFnpZZEa1Vz1PRYMNj3xCEhYZU7hAYQwyu1KBga3Lo0vEPXPSZW1o8DrM2C6IhYYGifzayZFNwZw5HtnPbyZvJfG4w6TX4EO8F0VSnZt87QtBW27nh5vSgRLN1XdzrVzm8h1ScZrPsIpSKJWVmNCWqOOeibloKfoZbhZ5A8vFz0I3nvMdi/v54DwcmS7GS/hryCgjhy4n9EhD1SnJ5325jnoyi4Fa5a/pibxPmAi8kU7ioHucmQRgv3yQHh17emqait9QNS4jTu6xyM6eeoVADsXTG4f7KK6nlZZjat";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    int objectsRecognized = 0;

    int level = 0;

    int xPosMarker = 150;

    int linearSlidelevel2 = 30;
    int linearSlidelevel3 = 160;

    int bucketTurnerlevel1 = -50;
    int bucketTurnerlevel2 = -70;
    int bucketTurnerlevel3 = -120;

    public void runOpMode() {

        if (initializeRobot() == 1) {
            telemetry.addData("Status", "Initialization failed..");
            return;
        }


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            telemetry.addData("Status", "Running");
            telemetry.update();

            double pval = 0.5D;

            objectDetection();
            strafeToShippingHub();
            placeFreightOnShippingHub();

            //getSensorValues();

            //strafeToCarousel(0.75D);

      /*
      while (opModeIsActive()) {
      telemetry.addData("bucket position: ", bucketTurner.getCurrentPosition());
      telemetry.addData("linear position: ", linearSlide.getCurrentPosition());
      telemetry.addData("level", level);
      telemetry.update();


      }
      */

        }

    }

    private int initializeRobot() {

        frontLeft = (DcMotor)hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = (DcMotor)hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = (DcMotor)hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = (DcMotor)hardwareMap.get(DcMotor.class, "BackRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





        bucket = (DcMotor)hardwareMap.get(DcMotor.class, "Bucket");
        bucketTurner = (DcMotor)hardwareMap.get(DcMotor.class, "BucketTurner");
        linearSlide = (DcMotor)hardwareMap.get(DcMotor.class, "LinearSlide");
        carouselTurner = (DcMotor)hardwareMap.get(DcMotor.class, "CarouselTurner");

        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bucketTurner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        FrontRightDistance = (DistanceSensor)hardwareMap.get(DistanceSensor.class, "FrontRightDistance");
        FrontLeftDistance = (DistanceSensor)hardwareMap.get(DistanceSensor.class, "FrontLeftDistance");
        BackRightDistance = (DistanceSensor)hardwareMap.get(DistanceSensor.class, "BackRightDistance");
        BackLeftDistance = (DistanceSensor)hardwareMap.get(DistanceSensor.class, "BackLeftDistance");
        FrontDistance = (DistanceSensor)hardwareMap.get(DistanceSensor.class, "FrontDistance");
        CarouselDistance = (DistanceSensor)hardwareMap.get(DistanceSensor.class, "CarouselDistance");
        FrontColor = (ColorSensor)hardwareMap.get(ColorSensor.class, "FrontColor");
        BackColor = (ColorSensor)hardwareMap.get(ColorSensor.class, "BackColor");

        carouselTurner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        bucket.setDirection(DcMotorSimple.Direction.FORWARD);
        bucketTurner.setDirection(DcMotorSimple.Direction.FORWARD);
        carouselTurner.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (initVuforia() == 1)
            return 1;
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0D, 2.0D);
        }
        return 0;
    }

    private void testEachWheel(double pval) {
        int start_enc_val = backRight.getCurrentPosition();
        backRight.setPower(pval);
        sleep(5000L);
        backRight.setPower(0.0D);
        sleep(1000L);
        int end_enc_val = backRight.getCurrentPosition();
        telemetry.addData("Back Right", " " + (end_enc_val - start_enc_val));
        telemetry.update();
    }

    private void testStrafing(double pval) {
        frontLeft.setPower(pval);
        backRight.setPower(pval);
        frontRight.setPower(-1.0D * pval);
        backLeft.setPower(-1.0D * pval);
        sleep(2000L);
        frontLeft.setPower(0.0D);
        frontRight.setPower(0.0D);
        backLeft.setPower(0.0D);
        backRight.setPower(0.0D);
        sleep(1000L);
    }

    private void testBucket(double pval) {
        bucket.setPower(pval);
        sleep(5000L);
        bucket.setPower(0.0D);
        sleep(1000L);
        bucket.setPower(-1.0D * pval);
        sleep(5000L);
        bucket.setPower(0.0D);
        sleep(1000L);
    }

    private void testBucketTurner(double pval) {
        bucketTurner.setPower(pval);
        sleep(1000L);
        bucketTurner.setPower(0.0D);
        sleep(1000L);
        bucketTurner.setPower(-1.0D * pval);
        sleep(1000L);
        bucketTurner.setPower(0.0D);
        sleep(1000L);
    }

    private void testLinearSlide(double pval) {
        linearSlide.setPower(pval);
        sleep(1000L);
        linearSlide.setPower(0.0D);
        sleep(1000L);
        linearSlide.setPower(-1.0D * pval);
        sleep(1000L);
        linearSlide.setPower(0.0D);
        sleep(1000L);
    }

    private void testCarouselTurner(double pval) {
        carouselTurner.setPower(pval);
        sleep(5000L);
        carouselTurner.setPower(0.0D);
        sleep(1000L);
        carouselTurner.setPower(-1.0D * pval);
        sleep(5000L);
        carouselTurner.setPower(0.0D);
        sleep(1000L);
    }

    private void linearSlideEncoder(double pval, int enCount) {
        linearSlide.setTargetPosition(enCount);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Motor Status: ", "Running");
        telemetry.update();
        linearSlide.setPower(pval);
    }

    private int initVuforia() {
        int vu_err = 0;
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = "Af2A/t3/////AAABmSsJTGsI6Ebgr6cIo4YGqmCBxd+lRenqxeIeJ3TQXcQgRlvrzKhb44K7xnbfJnHjD6eLQaFnpZZEa1Vz1PRYMNj3xCEhYZU7hAYQwyu1KBga3Lo0vEPXPSZW1o8DrM2C6IhYYGifzayZFNwZw5HtnPbyZvJfG4w6TX4EO8F0VSnZt87QtBW27nh5vSgRLN1XdzrVzm8h1ScZrPsIpSKJWVmNCWqOOeibloKfoZbhZ5A8vFz0I3nvMdi/v54DwcmS7GS/hryCgjhy4n9EhD1SnJ5325jnoyi4Fa5a/pibxPmAi8kU7ioHucmQRgv3yQHh17emqait9QNS4jTu6xyM6eeoVADsXTG4f7KK6nlZZjat";
        try {
            parameters.cameraName = (CameraName)hardwareMap.get(WebcamName.class, "Camera");
        } catch (Exception e) {
            telemetry.addData("System Error", "Error during hardware get Camera");
            vu_err = 1;
        }
        if (vu_err == 0)
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
        return vu_err;
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext
                .getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8F;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromFile("/sdcard/FIRST/tflitemodels/10820model.tflite", LABELS);
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
                        level = 2;
                        telemetry.addData("Level", Integer.valueOf(level));
                        telemetry.update();
                    } else if (leftVal >= xPosMarker && objectsRecognized == 1) {
                        level = 1;
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

    private void strafeToCarousel(double pval) {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setPower(pval);
        backRight.setPower(pval* -1.0);
        frontLeft.setPower(pval * -1.0);
        backLeft.setPower(pval);
        while (BackRightDistance.getDistance(DistanceUnit.INCH) >= 20.0D)
            idle();
        frontRight.setPower(0.0D);
        backRight.setPower(0.0D);
        frontLeft.setPower(0.0D);
        backLeft.setPower(0.0D);

        frontRight.setPower(0.25D);
        backRight.setPower(0.25D);
        frontLeft.setPower(0.25D);
        backLeft.setPower(0.25D);
        while (BackColor.alpha() <= 400)
            idle();
        frontRight.setPower(0.0D);
        backRight.setPower(0.0D);
        frontLeft.setPower(0.0D);
        backLeft.setPower(0.0D);
        bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketTurner.setTargetPosition(55);
        bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bucketTurner.setPower(0.75D);
        while (bucketTurner.isBusy())
            idle();
    }

    private void moveRobotForwardEncoders(double pval, int enCount) {

        resetEncoders();

        frontRight.setTargetPosition(enCount);
        backRight.setTargetPosition(enCount);
        frontLeft.setTargetPosition(enCount);
        backLeft.setTargetPosition(enCount);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Motor Status: ", "Running");
        telemetry.update();
        frontRight.setPower(pval);
        backRight.setPower(pval);
        frontLeft.setPower(pval);
        backLeft.setPower(pval);
        while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy())
            idle();
        frontRight.setPower(0.0D);
        backRight.setPower(0.0D);
        frontLeft.setPower(0.0D);
        backLeft.setPower(0.0D);
    }

    private void moveRobotBackwardEncoders(double pval, int enCount) {

        resetEncoders();

        frontRight.setTargetPosition(enCount * -1);
        backRight.setTargetPosition(enCount * -1);
        frontLeft.setTargetPosition(enCount * -1);
        backLeft.setTargetPosition(enCount * -1);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Motor Status: ", "Running");
        telemetry.update();
        frontRight.setPower(pval * -1.0D);
        backRight.setPower(pval * -1.0D);
        frontLeft.setPower(pval * -1.0D);
        backLeft.setPower(pval * -1.0D);
        while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy())
            idle();
        frontRight.setPower(0.0D);
        backRight.setPower(0.0D);
        frontLeft.setPower(0.0D);
        backLeft.setPower(0.0D);
    }

    private void strafeRobotRightEncoders(double pval, int enCount) {

        resetEncoders();

        frontRight.setTargetPosition(enCount * -1);
        backRight.setTargetPosition(enCount);
        frontLeft.setTargetPosition(enCount);
        backLeft.setTargetPosition(enCount * -1);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Motor Status: ", "Running");
        telemetry.update();
        frontRight.setPower(pval * -1.0D);
        backRight.setPower(pval);
        frontLeft.setPower(pval);
        backLeft.setPower(pval * -1.0D);
        while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy())
            idle();
        frontRight.setPower(0.0D);
        backRight.setPower(0.0D);
        frontLeft.setPower(0.0D);
        backLeft.setPower(0.0D);
    }

    private void strafeRobotLeftEncoders(double pval, int enCount) {
        resetEncoders();

        frontRight.setTargetPosition(enCount);
        backRight.setTargetPosition(enCount * -1);
        frontLeft.setTargetPosition(enCount * -1);
        backLeft.setTargetPosition(enCount);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Motor Status: ", "Running");
        telemetry.update();
        frontRight.setPower(pval);
        backRight.setPower(pval * -1.0D);
        frontLeft.setPower(pval * -1.0D);
        backLeft.setPower(pval);
        while (frontRight.isBusy() || frontLeft.isBusy() || backLeft.isBusy() || backRight.isBusy())
            idle();
        frontRight.setPower(0.0D);
        backRight.setPower(0.0D);
        frontLeft.setPower(0.0D);
        backLeft.setPower(0.0D);
    }

    private void getSensorValues() {
        while (opModeIsActive()) {
            telemetry.addData("Front Right Distance: ", Double.valueOf(FrontRightDistance.getDistance(DistanceUnit.INCH)));
            telemetry.addData("Front Left Distance: ", Double.valueOf(FrontLeftDistance.getDistance(DistanceUnit.INCH)));
            telemetry.addData("Back Right Distance: ", Double.valueOf(BackRightDistance.getDistance(DistanceUnit.INCH)));
            telemetry.addData("Back Left Distance: ", Double.valueOf(BackLeftDistance.getDistance(DistanceUnit.INCH)));
            telemetry.addData("Front Distance: ", Double.valueOf(FrontDistance.getDistance(DistanceUnit.INCH)));
            telemetry.addData("Carousel Distance: ", Double.valueOf(CarouselDistance.getDistance(DistanceUnit.INCH)));
            telemetry.addData("FrontColor Value: ", Integer.valueOf(FrontColor.alpha()));
            telemetry.addData("BackColor Value: ", Integer.valueOf(BackColor.alpha()));
            telemetry.update();
        }
    }

    private void moveBucketTurnerForwardEncoders(double pval, int enCount) {
        bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketTurner.setTargetPosition(enCount);
        bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bucketTurner.setPower(pval);
        while (bucketTurner.isBusy())
            idle();
        bucketTurner.setPower(0.0D);
    }

    private void moveBucketTurnerBackwardEncoders(double pval, int enCount) {
        bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketTurner.setTargetPosition(enCount * -1);
        bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bucketTurner.setPower(pval * -1.0D);
        while (bucketTurner.isBusy())
            idle();
        bucketTurner.setPower(0.0D);
    }

    private void moveBucketForwardEncoders(double pval, int enCount) {
        bucket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucket.setTargetPosition(enCount);
        bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bucket.setPower(pval);
        while (bucket.isBusy())
            idle();
        bucketTurner.setPower(0.0D);
    }

    private void moveBucketBackwardEncoders(double pval, int enCount) {
        bucket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucket.setTargetPosition(enCount * -1);
        bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bucket.setPower(pval * -1.0D);
        while (bucket.isBusy())
            idle();
        bucket.setPower(0.0D);
    }

    private void moveLinearSlideForwardEncoders(double pval, int enCount) {
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setTargetPosition(enCount);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(pval);
        while (linearSlide.isBusy())
            idle();
        linearSlide.setPower(0.0D);
    }

    private void moveLinearSlideBackwardEncoders(double pval, int enCount) {
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setTargetPosition(enCount * -1);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(pval * -1.0D);
        while (linearSlide.isBusy())
            idle();
        linearSlide.setPower(0.0D);
    }

    private void releaseItemLevel1() {

        bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketTurner.setTargetPosition(bucketTurnerlevel1);
        bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bucketTurner.setPower(-0.75D);
        while (bucketTurner.isBusy())
            idle();
        bucket.setPower(-0.75D);
        sleep(1500L);
        bucket.setPower(0.0D);
        moveRobotBackwardEncoders(0.75D, 600);

        telemetry.addData("Status: ", "Freight Placed");
        telemetry.update();
        frontRight.setPower(0.0D);
        backRight.setPower(0.0D);
        frontLeft.setPower(0.0D);
        backLeft.setPower(0.0D);
    }

    private void releaseItemLevel2() {
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setTargetPosition(linearSlidelevel2);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.75D);
        while (linearSlide.isBusy())
            idle();
        bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketTurner.setTargetPosition(bucketTurnerlevel2);
        bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bucketTurner.setPower(-0.75D);
        while (bucketTurner.isBusy())
            idle();
        bucket.setPower(-0.75D);
        sleep(1500L);
        bucket.setPower(0.0D);
        moveRobotBackwardEncoders(0.75D, 600);

    /*
    linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    linearSlide.setTargetPosition(-180);
    linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    linearSlide.setPower(-0.75D);
    while (linearSlide.isBusy())
      idle();
    linearSlide.setPower(0.0D);
    */

        telemetry.addData("Status: ", "Freight Placed");
        telemetry.update();
        frontRight.setPower(0.0D);
        backRight.setPower(0.0D);
        frontLeft.setPower(0.0D);
        backLeft.setPower(0.0D);
    }

    private void releaseItemLevel3() {
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setTargetPosition(linearSlidelevel3);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(0.75D);
        while (linearSlide.isBusy())
            idle();
        bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketTurner.setTargetPosition(bucketTurnerlevel3);
        bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bucketTurner.setPower(-0.75D);
        while (bucketTurner.isBusy())
            idle();
        bucket.setPower(-0.75D);
        sleep(1500L);
        bucket.setPower(0.0D);
        moveRobotBackwardEncoders(0.75D, 600);
        linearSlide.setPower(0);

    /*
    linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    linearSlide.setTargetPosition(-150);
    linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    linearSlide.setPower(-0.75D);
    while (linearSlide.isBusy())
      idle();
    linearSlide.setPower(0.0D);
    */

        telemetry.addData("Status: ", "Freight Placed");
        telemetry.update();
        frontRight.setPower(0.0D);
        backRight.setPower(0.0D);
        frontLeft.setPower(0.0D);
        backLeft.setPower(0.0D);
    }

    private void strafeToShippingHub() {

        resetEncoders();

        frontRight.setTargetPosition(-1500);
        backRight.setTargetPosition(1500);
        frontLeft.setTargetPosition(1500);
        backLeft.setTargetPosition(-1500);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setPower(-0.75D);
        backRight.setPower(0.75D);
        frontLeft.setPower(0.75D);
        backLeft.setPower(-0.75D);
        long curTime = System.currentTimeMillis();
        int timeRunning = 2000;
        long desiredTime = curTime + timeRunning;
        while (System.currentTimeMillis() <= desiredTime)
            idle();
        frontRight.setPower(0.0D);
        backRight.setPower(0.0D);
        frontLeft.setPower(0.0D);
        backLeft.setPower(0.0D);
        moveRobotForwardEncoders(0.5D, 600);
    }

    private void strafeLeft(double pval, int duration) {
        frontRight.setPower(pval);
        backRight.setPower(pval * -1.0D);
        frontLeft.setPower(pval * -1.0D);
        backLeft.setPower(pval);
        sleep(duration);
        frontRight.setPower(0.0D);
        backRight.setPower(0.0D);
        frontLeft.setPower(0.0D);
        backLeft.setPower(0.0D);
    }

    private void strafeRight(double pval, int duration) {
        frontRight.setPower(pval * -1.0D);
        backRight.setPower(pval);
        frontLeft.setPower(pval);
        backLeft.setPower(pval * -1.0D);
        sleep(duration);
        frontRight.setPower(0.0D);
        backRight.setPower(0.0D);
        frontLeft.setPower(0.0D);
        backLeft.setPower(0.0D);
    }

    private void placeFreightOnShippingHub() {
        if (level == 3) {
            telemetry.addData("Placing Freight on Level 3: ", "In Progress");
            telemetry.update();
            releaseItemLevel3();
        } else if (level == 2) {
            telemetry.addData("Placing Freight on Level 2: ", "In Progress");
            telemetry.update();
            releaseItemLevel2();
        } else if (level == 1) {
            telemetry.addData("Placing Freight on Level 1: ", "In Progress");
            telemetry.update();
            releaseItemLevel1();
        } else {
            telemetry.addData("No Objects Detected: ", "Strafing to Carousel");
            telemetry.update();
            strafeToCarousel(0.5D);
        }
        telemetry.addData("Carousel Turner Distance: ", Double.valueOf(CarouselDistance.getDistance(DistanceUnit.INCH)));
        telemetry.update();
    }

    private void resetEncoders() {

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        carouselTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    private void parkInWarehouse() {

        //turn

        frontRight.setPower(0.25D);
        backRight.setPower(0.25D);
        frontLeft.setPower(0.25D);
        backLeft.setPower(0.25D);

        //tune to white color
        while (BackColor.alpha() <= 400)
            idle();

        frontRight.setPower(0.0D);
        backRight.setPower(0.0D);
        frontLeft.setPower(0.0D);
        backLeft.setPower(0.0D);


    }

}


package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Red: States TeleOp", group = "Red")
public class RedStatesTeleOp extends LinearOpMode {

    //Motors
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private DcMotorEx carouselTurner;
    private DcMotorEx armMotor;
    private DcMotorEx bucketTurner;
    private DcMotor bucket;

    //Servos
    private Servo intakeServo;


	/*
	//Sensors
	private DistanceSensor distanceLeftFront;
	private DistanceSensor distanceLeftBack;
	private DistanceSensor distanceRightFront;
	private DistanceSensor distanceRightBack;
	private DistanceSensor distanceFront;
	private DistanceSensor distanceIntake;
	private ColorSensor colorFront;
	*/

    // Variables for the IMU
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;

    ElapsedTime carouselTimer = new ElapsedTime();

    ElapsedTime releaseTimer = new ElapsedTime();

    static double MAX_TICKS_PER_SECOND = 2000.0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("reached");
        telemetry.update();

        initializeRobot();

        waitForStart();

        while (opModeIsActive()) {

            double arm = -gamepad2.right_stick_y;
            armMotor.setVelocity(arm * RedStatesTeleOp.MAX_TICKS_PER_SECOND);

            double bucketVal = -gamepad2.left_stick_y;
            bucketTurner.setVelocity(bucketVal * RedStatesTeleOp.MAX_TICKS_PER_SECOND);

            if (gamepad2.dpad_up) {

                bucket.setPower(1);

            } else if (gamepad2.dpad_down) {

                bucket.setPower(-0.75);

            } else if (gamepad2.dpad_right) {

                bucket.setPower(0);

            }

			/*
			if (distanceIntake.getDistance(DistanceUnit.INCH) < 3) {

				telemetry.addLine("FREIGHT IN POSSESSION");
				telemetry.update();

			} else if (distanceIntake.getDistance(DistanceUnit.INCH) > 3) {

				telemetry.addLine("");
				telemetry.update();

			}
			*/


            if (gamepad2.x) {

                carouselTimer.reset();

                carouselTurner.setVelocity(1000);
                while (carouselTimer.milliseconds() < 500) {

                }

                carouselTurner.setVelocity(1250);
                while (carouselTimer.milliseconds() < 1000) {

                }

                carouselTurner.setVelocity(1750);
                while (carouselTimer.milliseconds() < 2250) {

                }

                carouselTurner.setVelocity(0);

            } else if (gamepad2.y) {

                carouselTimer.reset();

                carouselTurner.setVelocity(-1000);
                while (carouselTimer.milliseconds() < 500) {

                }

                carouselTurner.setVelocity(-1250);
                while (carouselTimer.milliseconds() < 1000) {

                }

                carouselTurner.setVelocity(-1750);
                while (carouselTimer.milliseconds() < 2250) {

                }

                carouselTurner.setVelocity(0);

            } else {

                carouselTurner.setVelocity(0);

            }

            if (gamepad2.right_bumper || gamepad2.left_bumper) {

                bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }


            if (gamepad1.y) {

                while (gamepad1.right_bumper || gamepad1.left_bumper != true) {

                    dropFreightInLevel(3);
                    break;

                }

                armMotor.setPower(0);
                bucketTurner.setPower(0);
                bucket.setPower(0);

                armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                bucketTurner.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            }

            if (gamepad1.x) {

                moveArmToEncoderVal(500, 0.75);
                moveBucketToEncoderVal(-1200, 0.75);
                rotateVelocity(-120,1500);
                intakeServo.setPosition(1.0);
                runBucketPower(-0.5);

                releaseTimer.reset();
                while (releaseTimer.milliseconds() < 1000) {

                }

                runBucketPower(0.0);
                intakeServo.setPosition(0.0);

                rotateVelocity(120,1500);

                armMotor.setPower(0);
                bucketTurner.setPower(0);
                bucket.setPower(0);

                armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                bucketTurner.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

                frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


            }

            if (gamepad1.dpad_up) {

                //pickup marker
                moveArmToEncoderVal(300, 1);
                moveBucketToEncoderVal(-650, 1);

                armMotor.setPower(0);
                bucketTurner.setPower(0);

                armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                bucketTurner.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            }


            //normal driver
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x ;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            setThrottle(frontLeftPower, frontRightPower, backLeftPower, backRightPower);

        }
    }

    public void setThrottle(double frontLeftVelocity, double frontRightVelocity, double backLeftVelocity, double backRightVelocity) {
        // Constrain throttle values to between -1.0 to 1.0
        if (frontLeftVelocity > 1.0) {

            frontLeftVelocity = 1.0;

        } else if (frontLeftVelocity < -1.0) {

            frontLeftVelocity = -1.0;

        }

        if (frontRightVelocity > 1.0) {

            frontRightVelocity = 1.0;

        } else if (frontRightVelocity < -1.0) {

            frontRightVelocity = -1.0;

        }

        if (backLeftVelocity > 1.0) {

            backLeftVelocity = 1.0;

        } else if (backLeftVelocity < -1.0) {

            backLeftVelocity = -1.0;

        }

        if (backRightVelocity > 1.0) {

            backRightVelocity = 1.0;

        } else if (backRightVelocity < -1.0) {

            backRightVelocity = -1.0;

        }

        // Set velocities
        frontLeft.setVelocity(frontLeftVelocity * RedStatesTeleOp.MAX_TICKS_PER_SECOND);
        frontRight.setVelocity(frontRightVelocity * RedStatesTeleOp.MAX_TICKS_PER_SECOND);
        backLeft.setVelocity(backLeftVelocity * RedStatesTeleOp.MAX_TICKS_PER_SECOND);
        backRight.setVelocity(backRightVelocity * RedStatesTeleOp.MAX_TICKS_PER_SECOND);
    }

    public void initializeRobot() {

        // initializing drive wheels
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        // initializing other motors
        bucket = hardwareMap.get(DcMotor.class, "Bucket");
        bucketTurner = hardwareMap.get(DcMotorEx.class, "BucketTurner");
        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        carouselTurner = hardwareMap.get(DcMotorEx.class, "CarouselTurner");

        //initialize servo
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

		/*
		// initializing sensors
		distanceLeftFront = hardwareMap.get(DistanceSensor.class, "distanceLeftFront");
		distanceLeftBack = hardwareMap.get(DistanceSensor.class, "distanceLeftBack");
		distanceRightFront = hardwareMap.get(DistanceSensor.class, "distanceRightFront");
		distanceRightBack = hardwareMap.get(DistanceSensor.class, "distanceRightBack");
		distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
		distanceIntake = hardwareMap.get(DistanceSensor.class, "distanceIntake");
		colorFront = hardwareMap.get(ColorSensor.class, "colorFront");
		*/
        initializeIMU();


        // sets the motors to use encoders
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bucketTurner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        carouselTurner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //reset encoders
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bucketTurner.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        carouselTurner.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // sets the motors to brake when there is no power applied (motor tries to actively maintain position)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucket.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucketTurner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselTurner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set direction of drive motors
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        //set direction of other motors
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        bucketTurner.setDirection(DcMotor.Direction.REVERSE);
        bucket.setDirection(DcMotor.Direction.FORWARD);
        carouselTurner.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized") ;
        telemetry.update() ;
    }

    public void dropFreightInLevel(int level) {

        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        bucketTurner.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        if (level == 1) {

            moveArmToEncoderVal(150, 0.5);
            moveBucketToEncoderVal(-650, 0.5) ;

        } else if (level == 2) {

            moveArmToEncoderVal(650, 0.5) ;
            moveBucketToEncoderVal(-1050, 0.5) ;

        } else if (level == 3) {
            moveArmToEncoderVal(950, 0.5) ;
            moveBucketToEncoderVal(-1150, 0.5) ;



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

    public void runBucketPower(double power) {

        bucket.setPower(power);

    }

    public void rotateVelocity(int degrees, double ticks) {
        double  leftVelocity, rightVelocity;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn left
            leftVelocity = -1.0 * ticks;
            rightVelocity = ticks;
        }
        else if (degrees > 0)
        {   // turn right
            leftVelocity = ticks;
            rightVelocity = -1.0*ticks;
        }
        else return;

        // set power to rotate.
        frontLeft.setVelocity(leftVelocity);
        backLeft.setVelocity(leftVelocity);
        frontRight.setVelocity(rightVelocity);
        backRight.setVelocity(rightVelocity);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (getAngle() == 0) {}

            while (getAngle() > degrees) {}
        }
        else	// left turn.
            while (getAngle() < degrees) {}

        // turn the motors off.
        stopDriveMotors();


        // reset angle tracking on new heading.
        resetAngle();
    }

    public void initializeIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode				= BNO055IMU.SensorMode.IMU;
        parameters.angleUnit		   = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit		   = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled	  = false;

        // get and initialize the IMU
        // The imu is assumed to be on I2C port
        // and configured to be a sensor of type "AdaFruit IMU" and named "imu"
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "IMU calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("imu calib status", "Calibrated" + imu.getCalibrationStatus().toString());
        telemetry.update( );
    }


    public void displayOrientationValues(Orientation orient, String name) {
        // ensure that the order is set up correctly as specified in getAngularDistance
        telemetry.addData(name, "Z: " + orient.firstAngle + ", Y: " + orient.secondAngle + ", X: " + orient.thirdAngle);
        telemetry.update() ;
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }


    private double checkDirection() {

        double angle, gain = .1;

        angle = getAngle();

        if (angle == 0)
            correction = 0;			 // no adjustment.
        else
            correction = -angle;		// reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void stopDriveMotors() {

        frontRight.setVelocity(0);
        frontLeft.setVelocity(0);
        backRight.setVelocity(0);
        backLeft.setVelocity(0);

    }
}

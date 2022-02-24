package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class StatesTeleOp extends LinearOpMode {

    //Motors
    private DcMotorEx frontRight;
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private DcMotorEx carouselTurner;
    private DcMotorEx armMotor;
    private DcMotorEx bucketTurner;
    private DcMotor bucket;

    ElapsedTime carouselTimer = new ElapsedTime();

    ElapsedTime releaseTimer = new ElapsedTime();

    static double MAX_TICKS_PER_SECOND = 2000.0;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();

        waitForStart();

        while (opModeIsActive()) {

            double arm = -gamepad2.right_stick_y;
            armMotor.setVelocity(arm * FSM_StatesTeleOp.MAX_TICKS_PER_SECOND);

            double bucketVal = -gamepad2.left_stick_y;
            bucketTurner.setVelocity(bucketVal * FSM_StatesTeleOp.MAX_TICKS_PER_SECOND);

            if (gamepad2.dpad_up) {

                bucket.setPower(1);

            } else if (gamepad2.dpad_down) {

                bucket.setPower(-0.75);

            } else if (gamepad2.dpad_right) {

                bucket.setPower(0);

            }

            double carouselTurnerForward = gamepad2.right_trigger;

            carouselTurner.setVelocity(carouselTurnerForward * FSM_StatesTeleOp.MAX_TICKS_PER_SECOND);

            double carouselTurnerBackward = -1 * gamepad2.left_trigger;

            carouselTurner.setVelocity(carouselTurnerBackward * FSM_StatesTeleOp.MAX_TICKS_PER_SECOND);

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

            if (gamepad1.a) {

                //lacuna
            }

            if (gamepad1.dpad_up) {

                //pickup marker
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
        frontLeft.setVelocity(frontLeftVelocity * FSM_StatesTeleOp.MAX_TICKS_PER_SECOND);
        frontRight.setVelocity(frontRightVelocity * FSM_StatesTeleOp.MAX_TICKS_PER_SECOND);
        backLeft.setVelocity(backLeftVelocity * FSM_StatesTeleOp.MAX_TICKS_PER_SECOND);
        backRight.setVelocity(backRightVelocity * FSM_StatesTeleOp.MAX_TICKS_PER_SECOND);
    }

    private void initializeRobot() {

        // initializing drive wheels
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        // sets the motors to use encoders
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        //reset encoders
        frontLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        //set direction of drive motors
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // sets the motors to brake when there is no power applied (motor tries to actively maintain position)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
}

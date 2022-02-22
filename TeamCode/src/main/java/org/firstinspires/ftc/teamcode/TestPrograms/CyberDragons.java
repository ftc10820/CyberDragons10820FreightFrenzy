package org.firstinspires.ftc.teamcode.TestPrograms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class CyberDragons extends CyberDragonsOpModeTemplate {

    public DcMotorEx frontRight;
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    //Arm + Other Motors
    public DcMotorEx bucket;
    public DcMotorEx bucketTurner;
    public DcMotorEx armMotor;
    public DcMotorEx carouselTurner;

    //Servos
    public Servo intakeServo;

    //Sensors
    public DistanceSensor distanceLeftFront;
    public DistanceSensor distanceLeftBack;
    public DistanceSensor distanceRightFront;
    public DistanceSensor distanceRightBack;
    public DistanceSensor distanceFront;
    public DistanceSensor distanceIntake;
    public ColorSensor colorFront;

    public CyberDragons(HardwareMap hardwareMap) {
        super();

        // initializing drive wheels
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");

        // initializing other motors
        bucket = hardwareMap.get(DcMotorEx.class, "Bucket");
        bucketTurner = hardwareMap.get(DcMotorEx.class, "BucketTurner");
        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        carouselTurner = hardwareMap.get(DcMotorEx.class, "CarouselTurner");

        //initialize servo
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        // initializing sensors
        distanceLeftFront = hardwareMap.get(DistanceSensor.class, "distanceLeftFront");
        distanceLeftBack = hardwareMap.get(DistanceSensor.class, "distanceLeftBack");
        distanceRightFront = hardwareMap.get(DistanceSensor.class, "distanceRightFront");
        distanceRightBack = hardwareMap.get(DistanceSensor.class, "distanceRightBack");
        distanceFront = hardwareMap.get(DistanceSensor.class, "distanceFront");
        distanceIntake = hardwareMap.get(DistanceSensor.class, "distanceIntake");
        colorFront = hardwareMap.get(ColorSensor.class, "colorFront");
        //initializeIMU() ;

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
        
        
    }
    
    
}

package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class PickUpMarker extends CyberDragonsOpModeTemplate {
  public void runOpMode() {
	long encoderArmValue;
    long encoderBucketValue;
    //initialization
	 initializeRobot();

	 waitForStart();
	 //run during op mode
    if(opModeIsActive()) {
      // long encoderArmValue;
      // long encoderBucketValue;
      moveArmWithBucket(15, 1, 230, 1);
      // encoderArmValue = armMotor.getCurrentPosition();
      // encoderBucketValue = bucketTurner.getCurrentPosition();
    }
  }
}

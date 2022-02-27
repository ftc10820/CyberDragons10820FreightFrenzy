/*
TeleOp:
1. Get infront of Team Marker
2. Press button or somehow changes to Autonomous
Autonomous:
1. Backup a enough to put the arm down
2. Move arm all the way down (Or enough to pick it up)
3. Move forward same amount as Part 1 or enough to connect to the hook thing
4. Move arm up to the top of the shipping area
5. Lower it enough so it touches the pole, and enough so it un-hooks
6. Move Back
7. Switch to TeleOp/End function
*/

package org.firstinspires.ftc.teamcode.TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
@Disabled
public class DropOffMarker extends CyberDragonsOpModeTemplate {
  public void runOpMode() {

	   //initialization
	    initializeRobot();

	    waitForStart();
	      //run during op mode
/*
  1st:
    Arm: 1056
    Bucket: -185
  2nd:
    Arm: 900
    Bucket: -89
*/
	if(opModeIsActive()) {
    sleep(2500);
      /*Temperary*/moveArmWithBucket(1056, .5, 250, .5);
      moveArmWithBucket(900, .5, 250, .5);
      moveArmWithBucket(845, .5, 113, .5);
      moveArmWithBucket(447, .5, 439, .5);
      sleep(20000);
    }//845, 113; 447, 439
  }
}

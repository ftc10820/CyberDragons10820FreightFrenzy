package org.firstinspires.ftc.teamcode.TestPrograms;

public class CoolMan extends CyberDragonsOpModeTemplate {

    @Override
    public void runOpMode() throws InterruptedException {


        initializeRobot();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        //while (opModeIsActive())
        if (opModeIsActive()) {
            //code here to run during the opmode, note that it is a while loop


        }
    }

}

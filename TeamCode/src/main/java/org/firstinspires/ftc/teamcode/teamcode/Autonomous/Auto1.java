package org.firstinspires.ftc.teamcode.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Autonomous", group = "Autonomous")


public class Auto1 extends AutonomousRobot {

    ElapsedTime runTime = new ElapsedTime();
    ElapsedTime waitTime = new ElapsedTime();

    @Override
    public void runOpMode(){

        super.runOpMode();

        waitForStart();


        //DriveByTime(2, .5, 90);

        //DriveByTime(2, .5, 180);

        TurnByTime(.25, -.5);

        DriveByTime(.5,.5,90);
        DriveByTime(.5,.5,180);
        DriveByTime(.5,.5,45);
        DriveByTime(1,.5,270);
        TurnByTime(.25, .5);





    }

}

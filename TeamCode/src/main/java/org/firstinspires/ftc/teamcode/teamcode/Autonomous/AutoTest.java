package org.firstinspires.ftc.teamcode.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.Autonomous.AutonomousRobot;


@Autonomous(name = "Strafe testing", group = "Autonomous")

@Disabled
public class AutoTest extends AutonomousRobot {



    @Override
    public void runOpMode(){

        super.runOpMode();

        waitForStart();


        //DriveByTime(2, .5, 90);

        //DriveByTime(2, .5, 180);

        TurnByTime(.25, -.5);
/*
        DriveByTime(.5,.5,90);
        DriveByTime(.5,.5,180);
        DriveByTime(.5,.5,45);
        DriveByTime(1,.5,270);
        TurnByTime(.25, .5);


*/


    }

}

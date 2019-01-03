package org.firstinspires.ftc.teamcode.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Depot Side Autonomous", group = "Autonomous")
public class AutoBotDepot extends AutonomousRobot{


    ElapsedTime runTime = new ElapsedTime();
    ElapsedTime waitTime = new ElapsedTime();

    double retractTime;


    @Override
    public void runOpMode(){

        super.runOpMode();

        setLocation(AutoPosition.DEPOT);

        //TELEMETRY SET-UP
        telemetry.addData("Is Aligned?: ", dogevuforia.getIsAligned()); // Is the bot aligned with the gold mineral?
        telemetry.addData("X Pos: ", dogevuforia.getGoldXPosition()); // Gold X position.
        telemetry.addData("Gold in sight: ", dogevuforia.isGold());//Is the gold cube in sight

        telemetry.update();

        SetServo();

        //dogevuforia.StartDoge();
        //WAIT FOR START
        waitForStart();

        //drive in front of right sample block
        DriveToSampling();

        runTime.reset();

        //Align and knock off gold cube.
        GoldAlign();

        retractTime = runTime.seconds();

        WaitFor(1);

        KnockGold();

        runTime.reset();

        DriveByTime(retractTime, .2, 90);


        DriveByTime(0.75, .6, 90);

        WaitFor(.25);

        TurnByTime(.765, .3);

        DriveByTime(.5, .6, 90);

        DriveByTime(3, .5, 180);

        PlaceMarker();

        //DriveByTime(2.2, .5, 0);

        //DriveByTime(.7, .6, 270);


    }

}

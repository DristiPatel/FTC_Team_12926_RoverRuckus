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


        waitForStart();

        //Unlatch();


        DriveToSampling();


        runTime.reset();

        GoldAlign();

        double retractTime = runTime.seconds();

        KnockGold();

        DriveByTime(retractTime, -.7);






    }


}

package org.firstinspires.ftc.teamcode.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AUto bots unite", group = "Autonomous")
public class AutoBot extends AutonomousRobot{


    ElapsedTime runTime = new ElapsedTime();
    ElapsedTime waitTime = new ElapsedTime();

    @Override
    public void runOpMode(){

        super.runOpMode();


        DriveByTime(2.0,.4,230);



        //Align and knock off gold cube.

        telemetry.addData("Is Aligned?: ", dogevuforia.getIsAligned()); // Is the bot aligned with the gold mineral?
        telemetry.addData("X Pos: ", dogevuforia.getGoldXPosition()); // Gold X position.
        telemetry.addData("Gold in sight: ", dogevuforia.isGold());//Is the gold cube in sight


        telemetry.update();

        //GoldAlign();

        StopVuforia();

    }

}

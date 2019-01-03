package org.firstinspires.ftc.teamcode.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Crater Side Autonomous", group = "Autonomous")
public class AutoBotCrater extends AutonomousRobot {


    ElapsedTime runTime = new ElapsedTime();

    double retractTime;

    @Override
    public void runOpMode(){

        super.runOpMode();

        setLocation(AutoPosition.CRATER);

        //TELEMETRY SET-UP
        //telemetry.addData("Is Aligned?: ", dogevuforia.getIsAligned()); // Is the bot aligned with the gold mineral?
        //telemetry.addData("X Pos: ", dogevuforia.getGoldXPosition()); // Gold X position.
        //telemetry.addData("Gold in sight: ", dogevuforia.isGold());//Is the gold cube in sight

        //telemetry.update();


        //SetServo();

        dogevuforia.StartDoge();
        //WAIT FOR START
        waitForStart();

        boolean run = true;

        try {

            while (run) {

                run = false;

                //drive in front of right sample block
                DriveToSampling();

                if(!opModeIsActive())
                    break;

                runTime.reset();
                //Align and knock off gold cube.
                GoldAlign();

                if(!opModeIsActive())
                    break;


                retractTime = runTime.seconds();

                WaitFor(1);
                //knock off gold, keep driving if going into crater, else stop in front of depot;

                if(!opModeIsActive())
                    break;


                KnockGold();

                if(!opModeIsActive())
                    break;


                runTime.reset();

                DriveByTime(retractTime, .2, 90);

                if(!opModeIsActive())
                    break;

/*
                //new code
                DriveByTime(2.15, .6, 270);

                if(!opModeIsActive())
                    break;


                TurnByTime(1.89, .3);

                if(!opModeIsActive())
                    break;


                DriveByTime(.45, .4, 90);

                if(!opModeIsActive())
                    break;


                DriveByTime(2, .6, 180);

                if(!opModeIsActive())
                    break;


                PlaceMarker();

                if(!opModeIsActive())
                    break;

                DriveByTime(4, .6, 0);

                if(!opModeIsActive())
                    break;


                idle();


*/
            }

        }catch(Exception e){robot.StopDriveMotors();}
        finally {
            stop();
        }

    }

}

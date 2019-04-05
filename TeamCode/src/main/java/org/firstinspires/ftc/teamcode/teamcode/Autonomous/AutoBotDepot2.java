package org.firstinspires.ftc.teamcode.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Depot Side Autonomous WITHOUT MARKER", group = "Autonomous")

@Disabled
public class AutoBotDepot2 extends BetterAutonomousRobot{


    ElapsedTime runTime = new ElapsedTime();
    ElapsedTime waitTime = new ElapsedTime();

    double retractTime;


    @Override
    public void runOpMode(){

        super.runOpMode();

        waitForStart();

        Unlatch();

        //get out of latch-----------------------
        EncoderDrive(1,-1,-1,1);

        rotate(-15, 1);

        EncoderDrive(1, 3, 3, 2);

        AbsoluteTurn(.5, baseAngle);

        WaitFor(.2);

        //sampling------------------------------------------------------------------
        Sampling();

        //turn towards wall and drive----------------------------------------------
        AbsoluteTurn(.7, baseAngle+53);

        EncoderDrive(1, 47, 47, 4);

        //turn toward depot and drive----------------------
        AbsoluteTurn(1, baseAngle+120);

       // EncoderDrive(1, -40, -40, 4);

        //deposit marker------------------------------------------------------------
        //Marker();

        //turn and drive to crater--------------------------------------------------
        //rotate(-5, 1);
        EncoderDrive(1, 10, 10, 4);

        //extend succ-----------------------------------------------------------------
        robot.extensionMotor.setPower(1);
        while (robot.extensionMotor.getCurrentPosition() < 900){
            idle();
        }
        robot.extensionMotor.setPower(0);

        robot.flipMotor.setPower(1);
        while(robot.flipMotor.getCurrentPosition() < 250){
            idle();
        }
        robot.flipMotor.setPower(0);


        dogevuforia.stop();

    }


}

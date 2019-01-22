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

        waitForStart();

        for (int i = 0; i < 1; i++) {

            int pos = robot.extensionMotor.getCurrentPosition();

            robot.extensionMotor.setTargetPosition(pos);
            robot.extensionMotor.setPower(.5);

            Unlatch();

            if (!opModeIsActive()){
                break;
            }

            DriveToSampling();

            if (!opModeIsActive()){
                break;
            }

            runTime.reset();

            GoldAlign();

            if (!opModeIsActive()){
                break;
            }

            AbsoluteTurn(.2, 0);
            AbsoluteTurn(.1, 0);

            if (!opModeIsActive()){
                break;
            }

            EncoderDrive(.8, 25, 25, 5);

        /*
        double retractTime = runTime.seconds();

        KnockGold();

        DriveByTime(retractTime, -.2);

        EncoderDrive(.5, -10, -10, 5);

        AbsoluteTurn(.1, 45);

        EncoderDrive(.5, -10, -10, 5);

        AbsoluteTurn(.1 , 135);

        PlaceMarker();
        */

        }

    }

}

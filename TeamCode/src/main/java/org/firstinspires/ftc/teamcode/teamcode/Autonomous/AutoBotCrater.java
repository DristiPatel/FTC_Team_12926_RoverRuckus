package org.firstinspires.ftc.teamcode.teamcode.Autonomous;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "Crater Side Autonomous", group = "Autonomous")

@Disabled
public class AutoBotCrater extends BetterAutonomousRobot {



    @Override
    public void runOpMode(){

        super.runOpMode();

        //SetServo();

        telemetry.addData("WORKING", true);
        telemetry.update();


        waitForStart();


        if(opModeIsActive()) {
            Unlatch();
        }
         //get out of hook---------------------------------------------------------

        if(opModeIsActive()) {
            EncoderDrive(1,-1,-1,1);
            rotate(-15, 1);
        }

        if(opModeIsActive()) {
            EncoderDrive(1, 3, 3, 2);
        }

        if(opModeIsActive()) {
            AbsoluteTurn(.5, baseAngle);
        }

        WaitFor(.2);

        //sampling-------------------------
        if(opModeIsActive()) {
            Sampling();
        }

        //turn toward wall and drive---------------------------------
        AbsoluteTurn(.7, baseAngle-115);

        EncoderDrive(1, -48, -48, 4);

        //turn toward depot and drive------------------------------------
        AbsoluteTurn(1, baseAngle-68);

        EncoderDrive(1, -50, -50, 5);

        //deposit marker------------------------------------
        Marker();

        //turn angled toward wall and toward crater and drive-----------------------
        AbsoluteTurn(1, baseAngle-43);
        EncoderDrive(1, 50, 50, 6);

        //extend arm into crater-----------------------------------------------
        robot.extensionMotor.setPower(1);
        while (robot.extensionMotor.getCurrentPosition() < 900 && opModeIsActive()){
            idle();
        }
        robot.extensionMotor.setPower(0);

        robot.flipMotor.setPower(1);
        while(robot.flipMotor.getCurrentPosition() < 250 && opModeIsActive()){
            idle();
        }
        robot.flipMotor.setPower(0);

        dogevuforia.StopDoge();

    }

}

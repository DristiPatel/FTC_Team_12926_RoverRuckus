package org.firstinspires.ftc.teamcode.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.HardwareRobot;


public class AutonomousRobot extends LinearOpMode {

    enum AutoPosition{

        CLOSE, FAR, UNKNOWN
    }

    HardwareRobot robot;
    AutoPosition location;
    ElapsedTime runTime;

    @Override
    public void runOpMode(){

        AutoPosition location = AutoPosition.UNKNOWN;

        robot = new HardwareRobot(hardwareMap);

        runTime = new ElapsedTime();

    }


    void WaitAbsolute(double seconds) {

        while (runTime.seconds() <= seconds && opModeIsActive()) {

            telemetry.addData("Time Remaining", Math.ceil(seconds - runTime.seconds()));
            //telemetry.update();

            idle();

        }
    }


    double getNewTime(double addedSeconds) {

        return runTime.seconds() + addedSeconds;
    }

    void WaitFor(double seconds) {

        WaitAbsolute(getNewTime(seconds));

    }




    //strafe drive for a given amount of seconds at a given speed and angle (in radians)
    void DriveByTime(double time, double speed, double angle){


       while(runTime.seconds() <= getNewTime(time)) {
           robot.frontLeft.setPower(speed*(Math.sin(-angle + Math.PI / 4)));
           robot.frontRight.setPower(speed*(Math.cos(-angle + Math.PI / 4)));
           robot.backLeft.setPower(speed*(Math.cos(-angle + Math.PI / 4)));
           robot.backRight.setPower(speed*(Math.sin(-angle + Math.PI / 4)));
       }

       robot.StopDriveMotors();


    }

    //speed from [-1 to 1]; negative is left and positive is right
    void TurnByTime(double time, double speed){

        while(runTime.seconds() <= getNewTime(time)) {
            robot.frontLeft.setPower(speed);
            robot.frontRight.setPower(-speed);
            robot.backLeft.setPower(speed);
            robot.backRight.setPower(-speed);
        }
        robot.StopDriveMotors();

    }



}

package org.firstinspires.ftc.teamcode.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.teamcode.HardwareRobot;


public class AutonomousRobot extends LinearOpMode {

    enum AutoPosition{

        CLOSE, FAR, UNKNOWN
    }

    enum GoldPosition{

        LEFT, CENTER, RIGHT, UNKNOWN;

    }

    HardwareRobot robot;
    ImuSensor imu;
    DogeVuforia dogevuforia;


    AutoPosition location = AutoPosition.UNKNOWN;
    GoldPosition goldPosition = GoldPosition.UNKNOWN;

    ElapsedTime runTime;


    @Override
    public void runOpMode(){

        location = AutoPosition.UNKNOWN;

        robot = new HardwareRobot(hardwareMap);
        imu = new ImuSensor(hardwareMap);
        dogevuforia = new DogeVuforia(hardwareMap);


        runTime = new ElapsedTime();



    }


    void WaitAbsolute(double seconds) {

        while (runTime.seconds() <= seconds) {

            //telemetry.addData("Time Remaining", Math.ceil(seconds - runTime.seconds()));
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


        angle = Math.toRadians(angle);

       robot.frontLeft.setPower(speed*(Math.cos(angle - Math.PI / 4)));
       robot.frontRight.setPower(speed*(Math.sin(angle - Math.PI / 4)));
       robot.backLeft.setPower(speed*(Math.sin(angle - Math.PI / 4)));
       robot.backRight.setPower(speed*(Math.cos(angle - Math.PI / 4)));

       WaitFor(time);

       robot.StopDriveMotors();


    }

    //speed from [-1 to 1]; negative is left and positive is right
    void TurnByTime(double time, double speed){


        robot.frontLeft.setPower(speed);
        robot.frontRight.setPower(-speed);
        robot.backLeft.setPower(speed);
        robot.backRight.setPower(-speed);

        WaitFor(time);

        robot.StopDriveMotors();

    }


    void AbsoluteTurn(double angle, double speed, double timeout) {



        robot.frontLeft.setPower(speed);
        robot.frontRight.setPower(-speed);
        robot.backLeft.setPower(speed);
        robot.backRight.setPower(-speed);

        double timeToStop = getNewTime(timeout);

        while (Math.abs(imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - angle) > 5 && runTime.seconds() <= timeToStop && opModeIsActive()) {

            telemetry.addData("Target Angle", angle)
                    .addData("Distance to go", Math.abs(imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - angle))
                    .addData("Current Angle", imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();

            idle();

        }

        WaitFor(0.25);

        robot.StopDriveMotors();

    }

    void RelativeTurn(double angle, double speed, double timeout) {

        double turnTo = angle + imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if (turnTo >= 180)
            turnTo -= 360;
        else if (turnTo <= -180)
            turnTo += 360;

        AbsoluteTurn(turnTo, speed, timeout);

    }


    void GoldAlign(){

        double stopTime = getNewTime(5);

        dogevuforia.StartDoge();

        while (runTime.seconds() <= stopTime) {

            while (!dogevuforia.getIsAligned()) {

                //move right until aligned
                robot.frontLeft.setPower(.2);
                robot.frontRight.setPower(.2);
                robot.backLeft.setPower(.2);
                robot.backRight.setPower(.2);

            }
            //stop robot if no gold cube in sight or aligned
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);

            break;

        }

        dogevuforia.StopDoge();

    }

    void KnockGold(){

        DriveByTime(.5, .3, 270);

        DriveByTime(1,.4,180);



    }



    void Unlatch(){


    }

    void Latch(){



    }

    void StopVuforia(){

        dogevuforia.StopDoge();

    }

}

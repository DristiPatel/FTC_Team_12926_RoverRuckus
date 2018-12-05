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

    //relative to crater
    enum AutoPosition{

        RED_CLOSE, RED_FAR, BLUE_CLOSE, BLUE_FAR, UNKNOWN
    }



    HardwareRobot robot;
    ImuSensor imu;
    DogeVuforia dogevuforia;


    AutoPosition location = AutoPosition.UNKNOWN;


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

    //rotate for a certain amount of time
    //speed from [-1 to 1]; negative is left and positive is right
    void TurnByTime(double time, double speed){


        robot.frontLeft.setPower(speed);
        robot.frontRight.setPower(-speed);
        robot.backLeft.setPower(speed);
        robot.backRight.setPower(-speed);

        WaitFor(time);

        robot.StopDriveMotors();

    }

    //rotate robot on center axis to an absolute angle
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

    //rotate robot on center axis for an angle relative to current position
    void RelativeTurn(double angle, double speed, double timeout) {

        double turnTo = angle + imu.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if (turnTo >= 180)
            turnTo -= 360;
        else if (turnTo <= -180)
            turnTo += 360;

        AbsoluteTurn(turnTo, speed, timeout);

    }

    void Unlatch(){


    }

    //drive to in front of the rightmost sampling block
    void DriveToSampling(){


        if(location == AutoPosition.RED_CLOSE || location == AutoPosition.BLUE_CLOSE) {
            DriveByTime(1.25, .4, 0);
            DriveByTime(1.3, .4, 270);
        }else{
            DriveByTime(1.25, .4, 0);
            DriveByTime(1.3, .4, 90);

        }
    }

    //keep driving left until aligned with the gold block
    void GoldAlign(){

        double stopTime = getNewTime(5);

        dogevuforia.StartDoge();


        while (!dogevuforia.getIsAligned() && runTime.seconds() <= stopTime) {

            if(location == AutoPosition.RED_CLOSE || location == AutoPosition.BLUE_CLOSE) {
                //move left until aligned
                robot.frontLeft.setPower(.2);
                robot.frontRight.setPower(.2);
                robot.backLeft.setPower(.2);
                robot.backRight.setPower(.2);
            }else{
                //move right until aligned
                robot.frontLeft.setPower(-.2);
                robot.frontRight.setPower(-.2);
                robot.backLeft.setPower(-.2);
                robot.backRight.setPower(-.2);

            }
        }

        //stop robot if no gold cube in sight or aligned
        robot.StopDriveMotors();

        dogevuforia.StopDoge();

    }

    //drive forward to knock off gold cube and drive back
    void KnockGold(){

        if (location == AutoPosition.RED_CLOSE || location == AutoPosition.BLUE_CLOSE) {

            DriveByTime(3, .4, 0);

        }else{



        }
    }


    void DriveToCenter(){

        if (location == AutoPosition.RED_CLOSE || location == AutoPosition.BLUE_CLOSE){

            while (!dogevuforia.isTargetVisible()){

                robot.frontLeft.setPower(.2);
                robot.frontRight.setPower(.2);
                robot.backLeft.setPower(.2);
                robot.backRight.setPower(.2);

            }


        }else{

            robot.frontLeft.setPower(-.2);
            robot.frontRight.setPower(-.2);
            robot.backLeft.setPower(-.2);
            robot.backRight.setPower(-.2);
        }

        robot.StopDriveMotors();
    }


    void Latch(){



    }

    void setLocation(AutoPosition pos){

        location = pos;

    }


}

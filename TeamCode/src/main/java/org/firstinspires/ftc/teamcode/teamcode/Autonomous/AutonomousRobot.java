package org.firstinspires.ftc.teamcode.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.teamcode.HardwareRobot;


public class AutonomousRobot extends LinearOpMode {

    //relative to crater
    enum AutoPosition{

        CRATER, DEPOT , UNKNOWN
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



        while (opModeIsActive() && runTime.seconds() <= seconds) {

            if(!opModeIsActive()){

                robot.StopDriveMotors();
                break;

            }

            //telemetry.addData("Time Remaining", Math.ceil(seconds - runTime.seconds()));
            //telemetry.update();
            telemetry.addData("Running", runTime.seconds());
            telemetry.update();
            idle();

        }

        if(!opModeIsActive())
            stop();



    }

    double getNewTime(double addedSeconds) {

        return runTime.seconds() + addedSeconds;
    }

    void WaitFor(double seconds) {

        WaitAbsolute(getNewTime(seconds));

    }


    //strafe drive for a given amount of seconds at a given speed and angle (in radians)
    // OUTDATED*******************************************************************
    void DriveByTime(double time, double speed, double angle){


        angle = Math.toRadians(angle);

       robot.leftDrive.setPower(speed*(Math.cos(angle - Math.PI / 4)));
       robot.rightDrive.setPower(speed*(Math.sin(angle - Math.PI / 4)));


       WaitFor(time);

       robot.StopDriveMotors();


    }

    //rotate for a certain amount of time
    //speed from [-1 to 1]; negative is left and positive is right
    void TurnByTime(double time, double speed){


        robot.leftDrive.setPower(speed);
        robot.rightDrive.setPower(-speed);


        WaitFor(time);

        robot.StopDriveMotors();

    }

    //rotate robot on center axis to an absolute angle
    void AbsoluteTurn(double angle, double speed, double timeout) {



        robot.leftDrive.setPower(speed);
        robot.rightDrive.setPower(-speed);


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

    // ENCODER METHODS----------------------------------------------------

    //encoder constants
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.54331 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 1;


    void EncoderDrive(double speed, double leftInches, double rightInches, double timeout){

        ElapsedTime runtime = new ElapsedTime();

        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }


    }

    void StrafeDrive(double speed, double inches, double timeout){

        ElapsedTime runtime = new ElapsedTime();

        int newStrafeTarget;

        if (opModeIsActive()){

            newStrafeTarget = robot.leftDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            robot.strafeDrive.setTargetPosition(newStrafeTarget);

            robot.strafeDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            robot.strafeDrive.setPower(Math.abs(speed));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (robot.strafeDrive.isBusy())) {

              idle();
            }

            // Stop all motion;
            robot.strafeDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.strafeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move


        }

    }





    void Unlatch(){

        final int MAX_POS = 500;

        while(robot.liftMotor.getCurrentPosition() < MAX_POS){

            robot.liftMotor.setPower(.3);
        }
        robot.liftMotor.setPower(0);

        //move out of latch

    }

    void StartDoge(){

        dogevuforia.StartDoge();
    }

    //drive to in front of the rightmost sampling block

    void DriveToSampling(){

        RelativeTurn(90,.3,4);
        StrafeDrive(.4,15, 4);
        EncoderDrive(.5,-15,-15, 4);

    }

    //keep driving left until aligned with the gold block

    void GoldAlign(){

        double stopTime = getNewTime(4.25);


        StartDoge();

        while (!dogevuforia.getIsAligned() && runTime.seconds() <= stopTime) {

                robot.leftDrive.setPower(.3);
                robot.rightDrive.setPower(.3);

              idle();
        }

        //stop robot if no gold cube in sight or aligned
        robot.StopDriveMotors();

        dogevuforia.StopDoge();

    }

    //drive forward to knock off gold cube, keep driving if going into crater, otherwise stop and place marker (unfinished)
    // OUTDATED*******************************************************************

    void KnockGold(){

        StrafeDrive(.6,10,4);
        StrafeDrive(.6,-10,4);


    }


    void SetServo(){

        robot.markerServo.setPosition(Servo.MAX_POSITION);

    }

    void PlaceMarker(){

        robot.markerServo.setPosition(Servo.MIN_POSITION);

        WaitFor(.9);

        robot.markerServo.setPosition(Servo.MAX_POSITION);

        WaitFor(.9);

        robot.markerServo.setPosition(Servo.MIN_POSITION);

        WaitFor(.5);

        robot.markerServo.setPosition(Servo.MAX_POSITION);

    }

    void setLocation(AutoPosition pos){

        location = pos;

    }


}

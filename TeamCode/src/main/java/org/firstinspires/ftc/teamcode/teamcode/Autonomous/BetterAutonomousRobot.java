package org.firstinspires.ftc.teamcode.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcode.BetterHardwareRobot;
import org.firstinspires.ftc.teamcode.teamcode.HardwareRobot;

import java.sql.Driver;


public class BetterAutonomousRobot extends LinearOpMode {


    //declare stuff
    BetterHardwareRobot robot;
    DogeVuforia dogevuforia;
    ElapsedTime runTime;

    //imu instance fields
    BNO055IMU               imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    double baseAngle;


    public enum GoldPosition{

        LEFT, MID, RIGHT;
    }


    @Override
    public void runOpMode(){


        //initialize and map the robot hardware
        robot = new BetterHardwareRobot(hardwareMap);

        //initialize the dogeforia for webcam
        dogevuforia = new DogeVuforia(hardwareMap);

        //change this
        //dogevuforia.setAlignmentSettings(0, 100);

        //internal runtime for time algorithms
        runTime = new ElapsedTime();

        //reset servo position and reset motor encoders
        robot.ResetAllEncoders();
        //robot.ResetServos();

        //IMU STUFF
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //wait for gyro to calibrate

        while (!imu.isGyroCalibrated()) {

            telemetry.addLine("Calibrating...");
            telemetry.addData("heading: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
        }

         //READY

        telemetry.addLine("Gyro ready");
        telemetry.addData("heading: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();

        baseAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        robot.markerServo.setPosition(Servo.MIN_POSITION);
        robot.webcamServo.setPosition(Servo.MAX_POSITION);

        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //TIME METHODS-------------------------------------------------------------------------------
    void WaitAbsolute(double seconds) {

        while (opModeIsActive() && runTime.seconds() <= seconds) {
            if(!opModeIsActive()){
                robot.StopDriveMotors();
                break;
            }
            telemetry.addData("Time Remaining ", Math.ceil(seconds - runTime.seconds()));
            telemetry.update();
            telemetry.addData("Current Time ", runTime.seconds());
            telemetry.update();
            idle();
        }
        if(!opModeIsActive())
            stop();
    }

    double getNewTime(double addedSeconds) {
        return runTime.seconds() + addedSeconds;
    }

    /**
     * wait for a given time in seconds
     * @param seconds
     */
    void WaitFor(double seconds) {
        WaitAbsolute(getNewTime(seconds));
    }


    //TIME NAVIGATION METHODS--------------------------------------------------------------------------
    public void TimeDrive(double angle, double time, double power){

        angle = Math.toRadians(angle) - Math.PI/4;

        if (opModeIsActive()) {
            robot.frontLeft.setPower((power * Math.cos(angle)));
            robot.frontRight.setPower((power * Math.sin(angle)));
            robot.backLeft.setPower((power * Math.sin(angle)));
            robot.backRight.setPower((power * Math.cos(angle)));
        }

        WaitFor(time);
        robot.StopDriveMotors();

    }

    public void StrafeTime(double power, double time){

        if(opModeIsActive()) {
            robot.backLeft.setPower(power);
            robot.backRight.setPower(-power);
            robot.frontLeft.setPower(power);
            robot.frontRight.setPower(-power);

        }
        WaitFor(time);
        robot.StopDriveMotors();
    }

    public void TurnByTime(double time, double speed) {

        if (opModeIsActive()) {
            robot.frontLeft.setPower(speed);
            robot.backRight.setPower(speed);
            robot.frontRight.setPower(-speed);
            robot.backRight.setPower(-speed);
        }

        WaitFor(time);

        robot.StopDriveMotors();

    }

    //ENCODER NAVIGATION METHODS--------------------------------------------------------------------------
    //encoder constants
    static final double     COUNTS_PER_MOTOR_REV    = 2240 ;    // change for mecanum
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    void EncoderDrive(double speed, double leftInches, double rightInches, double timeout) {
        ElapsedTime runtime = new ElapsedTime();

        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.frontLeft.getCurrentPosition() + (int) ((leftInches * COUNTS_PER_INCH)/8);
            newRightTarget = robot.frontRight.getCurrentPosition() + (int) ((rightInches * COUNTS_PER_INCH)/8);
            robot.frontLeft.setTargetPosition(newLeftTarget);
            robot.backLeft.setTargetPosition(newLeftTarget);
            robot.frontRight.setTargetPosition(newRightTarget);
            robot.backRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            robot.frontLeft.setPower(Math.abs(speed));
            robot.backLeft.setPower(Math.abs(speed));
            robot.frontRight.setPower(Math.abs(speed));
            robot.frontRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.StopDriveMotors();

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    //IMU GYROSCOPE NAVIGATION METHODS--------------------------------------------------------------------------

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    public void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.

        robot.backLeft.setPower(leftPower);
        robot.frontLeft.setPower(leftPower);
        robot.backRight.setPower(rightPower);
        robot.frontRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {

                telemetry.addData("current angle: ", getAngle());
                telemetry.addData("target angle: ", degrees);
                telemetry.update();
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {

                telemetry.addData("current angle: ", getAngle());
                telemetry.addData("target angle: ", degrees);
                telemetry.update();
            }

        // turn the motors off.
        robot.StopDriveMotors();

        // wait for rotation to stop.
        //sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    void AbsoluteTurn(double speed, double targetAngle){

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        if (currentAngle < targetAngle){

            while (opModeIsActive() && imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < targetAngle) {

                robot.frontLeft.setPower(-speed);
                robot.backLeft.setPower(-speed);
                robot.frontRight.setPower(speed);
                robot.backRight.setPower(speed);
            }


        }else if (currentAngle > targetAngle){

            while (opModeIsActive() && imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > targetAngle) {

                robot.frontLeft.setPower(speed);
                robot.backLeft.setPower(speed);
                robot.frontRight.setPower(-speed);
                robot.backRight.setPower(-speed);
            }
        }

        robot.StopDriveMotors();

    }

    //VUFORIA NAVIGATION METHODS--------------------------------------------------------------------------

    public void Unlatch(){

        //change
        final int UNLATCH_POS = 9035;

        robot.liftMotor.setPower(1);

        //unleash the webcam
        robot.webcamServo.setPosition(0);

        while (opModeIsActive() && robot.liftMotor.getCurrentPosition() < UNLATCH_POS) {

            telemetry.addData("Speed", robot.liftMotor.getPower());
            idle();

        }

        robot.liftMotor.setPower(0);

        //TimeDrive(90, .25, .5);
    }

    public void Succ(){

        //lower succ
        while (robot.flipMotor.getCurrentPosition() < 150) {
            robot.flipMotor.setPower(.4);
        }
        robot.flipMotor.setPower(0);

        while (robot.extensionMotor.getCurrentPosition() < 900){

            robot.extensionMotor.setPower(1);
        }
        robot.extensionMotor.setPower(0);


        robot.intakeServo.setPower(1);

        WaitFor(2);

        robot.intakeServo.setPower(0);

        while (robot.extensionMotor.getCurrentPosition() > 20){

            robot.extensionMotor.setPower(-1);
        }
        robot.extensionMotor.setPower(0);

        while (robot.flipMotor.getCurrentPosition() > 20){

            robot.flipMotor.setPower(-.3);
        }

    }

    public void Sampling(){

        dogevuforia.start();
        WaitFor(.5);

        GoldPosition goldPos = GoldPosition.RIGHT;

        //select gold position

        //check the left and mid, otherwise choose right
        if (dogevuforia.isGold() && dogevuforia.getPosition().y > 125){
            if(dogevuforia.getGoldXPosition() > 100 && dogevuforia.getGoldXPosition() < 250){
                goldPos = GoldPosition.LEFT;
            }else if (dogevuforia.getGoldXPosition() > 425 && dogevuforia.getGoldXPosition() < 600){
                goldPos = GoldPosition.MID;
            }

        }else{

            goldPos = GoldPosition.RIGHT;
        }

        /*
        while (runTime.seconds() < getNewTime(1)) {
            telemetry.addData("Gold pos:", goldPos);
            telemetry.update();
        }
        */
        switch(goldPos){

            case LEFT:
                rotate(30, 1);
                //Succ();
                EncoderDrive(1,27, 27, 3);
                EncoderDrive(1,-27, -27, 3);

                //AbsoluteTurn(.5, baseAngle);
                break;

            case MID:
                //WaitFor(2);
                //Succ();
                EncoderDrive(1,25, 25, 3);
                EncoderDrive(1,-25, -25, 3);
                break;

            case RIGHT:
                rotate(-30, 1);
                //Succ();
                EncoderDrive(1,27, 27, 3);
                EncoderDrive(1,-27, -27, 3);
                //AbsoluteTurn(.5, baseAngle);
        }

    }

    public void Marker(){

        robot.markerServo.setPosition(Servo.MAX_POSITION);

        WaitFor(1);

        robot.markerServo.setPosition(Servo.MIN_POSITION);


    }

}

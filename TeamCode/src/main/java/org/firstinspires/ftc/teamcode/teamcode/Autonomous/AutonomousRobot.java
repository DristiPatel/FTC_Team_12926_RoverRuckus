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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcode.HardwareRobot;



public class AutonomousRobot extends LinearOpMode {

    //relative to crater
    enum AutoPosition{

        CRATER, DEPOT , UNKNOWN
    }



    HardwareRobot robot;

    DogeVuforia dogevuforia;


    AutoPosition location;


    ElapsedTime runTime;

    //imu variables
    BNO055IMU               imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;


    @Override
    public void runOpMode(){

        location = AutoPosition.UNKNOWN;

        robot = new HardwareRobot(hardwareMap);
        dogevuforia = new DogeVuforia(hardwareMap);

        runTime = new ElapsedTime();

        robot.ResetAllEncoders();
        robot.ResetServos();

        robot.extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extensionMotor.setTargetPosition(robot.extensionMotor.getCurrentPosition());

        //imu stuffs
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
    void DriveByTime(double time, double speed){



       robot.leftDrive.setPower(speed);
       robot.rightDrive.setPower(speed);


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
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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
            robot.strafeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.strafeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move


        }

    }

    void TimeStrafe(double speed, double time){


        if (opModeIsActive()) {
            robot.strafeDrive.setPower(speed);
            WaitFor(time);
        }
        robot.StopDriveMotors();


    }

    //initialize servo positions
    void SetServo(){

        robot.markerServo.setPosition(Servo.MAX_POSITION);
        robot.webcamServo.setPosition(Servo.MAX_POSITION);


    }

    //IMU METHODS---------------------------------------------------------------------------

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.thirdAngle - lastAngles.thirdAngle;

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
    private double checkDirection()
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
    private void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    //180 to -180 going left to right
    void AbsoluteTurn(double speed, double targetAngle){

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        if (currentAngle < targetAngle){

            while (opModeIsActive() && imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < targetAngle) {

                robot.leftDrive.setPower(-speed);
                robot.rightDrive.setPower(speed);
            }


        }else if (currentAngle > targetAngle){

            while (opModeIsActive() && imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > targetAngle) {

                robot.leftDrive.setPower(speed);
                robot.rightDrive.setPower(-speed);
            }
        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);

    }

    //PATHING METHODS
   // --------------------------------------------------------------

    void Unlatch(){

        final int UNLATCH_POS = -4076;



        while (opModeIsActive() && robot.liftMotor.getCurrentPosition() > UNLATCH_POS) {

            robot.liftMotor.setPower(-1);



    }

        robot.liftMotor.setPower(0);


        EncoderDrive(.1, -1, -1, 2);


    }



    //drive out of hook
    void DriveToSampling(){


        StrafeDrive(.5,-9,5);

        EncoderDrive(.5,5,5,5);

        StrafeDrive(.5,9,5);


        AbsoluteTurn(.2, 50);
        AbsoluteTurn(.1, 55);

        EncoderDrive(.4, 22, 22, 5);

        AbsoluteTurn(.2, -90);
        AbsoluteTurn(.1, -90);


        /**
        AbsoluteTurn(.1,0);
        AbsoluteTurn(.1,0);


        EncoderDrive(.6,14,14,5);

        AbsoluteTurn(.2, -90);
        AbsoluteTurn(.1, -90);


        sleep(250);

        //drive to leftmost
        EncoderDrive(.7, -30, -30, 4);
        sleep(250);
*/

    }


    //turn all the way left, then keep turning right until aligned with gold block

    void GoldAlign(){


        //actual alignment
        dogevuforia.StartDoge();

        double stopTime = getNewTime(4);

        while (!dogevuforia.getIsAligned() && runTime.seconds() <= stopTime){

            robot.leftDrive.setPower(.2);
            robot.rightDrive.setPower(.2);

        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        sleep(250);


    }

    void KnockGold(){

/*
        AbsoluteTurn(.1, 0);
        AbsoluteTurn(.1, 0);

        EncoderDrive(.6, 10, 10, 4);
        EncoderDrive(.6, -10, -10, 4);

        AbsoluteTurn(.2, -90);
        AbsoluteTurn(.1, -90);
        //AbsoluteTurn(.3,-50);
        //AbsoluteTurn(.1, -90);
*/

        TimeStrafe(-.7, .5);
        TimeStrafe(.7, .5);



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

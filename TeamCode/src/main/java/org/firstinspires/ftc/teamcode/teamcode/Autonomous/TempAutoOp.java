package org.firstinspires.ftc.teamcode.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcode.Autonomous.DogeVuforia;

import java.sql.Time;

@Autonomous(name="Weekend Auto", group="Autonomous")
public class TempAutoOp extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private DcMotor latchMotor, slideMotor, intakeMotor;

    private CRServo lflipServo, rflipServo;

    private BNO055IMU imu;

    private DogeVuforia dogevuforia;

    private ElapsedTime runTime;

    public enum GoldPosition{

        LEFT, MID, RIGHT;
    }


    private double baseAngle;
    @Override
    public void runOpMode(){


        //map hardware---------------------------------------------
        frontLeft = hardwareMap.dcMotor.get("fl");
        frontRight = hardwareMap.dcMotor.get("fr");
        backLeft = hardwareMap.dcMotor.get("bl");
        backRight = hardwareMap.dcMotor.get("br");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        latchMotor = hardwareMap.dcMotor.get("latch");
        slideMotor = hardwareMap.dcMotor.get("slide");
        intakeMotor = hardwareMap.dcMotor.get("intake");

        lflipServo = hardwareMap.crservo.get("lflip");
        rflipServo = hardwareMap.crservo.get("rflip");


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        // parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu.initialize(parameters);

        ResetEncoders();

        dogevuforia = new DogeVuforia(hardwareMap);

        runTime = new ElapsedTime();

        while (!imu.isGyroCalibrated()) {

            telemetry.addLine("Calibrating...");
            telemetry.addData("heading: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
        }

        baseAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        //START-------------------------

        waitForStart();

        //get out of latch

        EncoderDrive(1, -2,-2,10);
        TimeStrafe(1,.6);
        EncoderDrive(.5, 4,4, 10);

        //sample
        Sampling();

        //turn to wall and drive
        AbsoluteTurn(.1, baseAngle -25);
        EncoderDrive(1, 20,20,10);

        //turn to depot and drive
        AbsoluteTurn(.1, baseAngle-130);

        EncoderDrive(1, 10, 10, 10);

        //release marker
        lflipServo.setPower(1);
        rflipServo.setPower(-1);

        WaitFor(2);

        lflipServo.setPower(0);
        rflipServo.setPower(0);

        intakeMotor.setPower(-1);

        WaitFor(.75);

        intakeMotor.setPower(0);

        lflipServo.setPower(-1);
        rflipServo.setPower(1);

        WaitFor(2);


        //drive into crater
        EncoderDrive(1, -40,-40, 10);



    }

    //TIME METHODS-------------------------------------------------------------------------------
    void WaitAbsolute(double seconds) {

        while (opModeIsActive() && runTime.seconds() <= seconds) {
            if(!opModeIsActive()){
                StopDriveMotors();
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


    //METHODS---------------------------------------------------------------

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // change for mecanum
    static final double     DRIVE_GEAR_REDUCTION    = 1.5 ;     // This is < 1.0 if geared UP
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
            newLeftTarget = frontLeft.getCurrentPosition() + (int) ((leftInches * COUNTS_PER_INCH));
            newRightTarget = frontRight.getCurrentPosition() + (int) ((rightInches * COUNTS_PER_INCH));
            frontLeft.setTargetPosition(newLeftTarget);
            backLeft.setTargetPosition(newLeftTarget);
            frontRight.setTargetPosition(newRightTarget);
            backRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (frontLeft.isBusy() && frontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        frontLeft.getCurrentPosition(),
                        frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            StopDriveMotors();

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    void TimeStrafe(double speed, double time){

        frontLeft.setPower(speed);
        backLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backRight.setPower(speed);

        WaitFor(time);

        StopDriveMotors();

    }
    void AbsoluteTurn(double speed, double targetAngle){

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        if (currentAngle < targetAngle){

            while (opModeIsActive() && imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < targetAngle) {

                frontLeft.setPower(-speed);
                backLeft.setPower(-speed);
                frontRight.setPower(speed);
                backRight.setPower(speed);
            }


        }else if (currentAngle > targetAngle){

            while (opModeIsActive() && imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > targetAngle) {

                frontLeft.setPower(speed);
                backLeft.setPower(speed);
                frontRight.setPower(-speed);
                backRight.setPower(-speed);
            }
        }

        StopDriveMotors();

    }


    private void ResetDriveEncoders(){


        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void StopDriveMotors(){

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    public void ResetEncoders(){

        ResetDriveEncoders();
        latchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        latchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void Sampling(){

        //dogevuforia.start();
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


      /* while (runTime.seconds() < getNewTime(5)) {
            telemetry.addData("Gold pos:", goldPos);
            telemetry.update();

        }
*/
        switch(goldPos){

            case LEFT:

                EncoderDrive(1,-4,-4,10);
                TimeStrafe(1,1.5);
                TimeStrafe(1,-1.5);
                EncoderDrive(1,4,4,10);
                break;

            case MID:
                TimeStrafe(1,1.5);
                TimeStrafe(1,-1.5);
                break;

            case RIGHT:
                EncoderDrive(1,4,4,10);
                TimeStrafe(1,1.5);
                TimeStrafe(1,-1.5);
                EncoderDrive(1,-4,-4,10);
                break;
        }

    }

}

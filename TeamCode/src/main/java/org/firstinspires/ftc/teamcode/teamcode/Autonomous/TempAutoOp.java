package org.firstinspires.ftc.teamcode.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.teamcode.Autonomous.DogeVuforia;

import java.sql.Time;

@Autonomous(name="Weekend Auto", group="Autonomous")
public class TempAutoOp extends LinearOpMode {

    private DcMotor latchMotor, slideMotor, intakeMotor, liftMotor,
            leftBack, rightBack, leftFront, rightFront;

    private Servo blockServo, boxServo;
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

        latchMotor = hardwareMap.dcMotor.get("latch");
        slideMotor = hardwareMap.dcMotor.get("slide");
        intakeMotor = hardwareMap.dcMotor.get("intake");
        liftMotor = hardwareMap.dcMotor.get("lift");

        leftBack = hardwareMap.dcMotor.get("lb");
        rightBack = hardwareMap.dcMotor.get("rb");
        leftFront = hardwareMap.dcMotor.get("lf");
        rightFront = hardwareMap.dcMotor.get("rf");

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        latchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        latchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lflipServo = hardwareMap.crservo.get("flipl");
        rflipServo = hardwareMap.crservo.get("flipr");
        blockServo = hardwareMap.servo.get("block");
        boxServo = hardwareMap.servo.get("box");


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

        latchMotor.setPower(-0.001);

        //START-------------------------

        waitForStart();

        latchMotor.setPower(1);
        WaitFor(2.8);
        latchMotor.setPower(0);

        Drive(0, -1, 0, 0.3);
        WaitFor(0.2);
        StopDriveMotors();

        WaitFor(0.50);

        Sampling();

        //get out of latch

        /*EncoderDrive(1, -2,-2,10);
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
        */


    }

    void Drive(double x, double y, double rot, double speedMod){

        double dx = x;
        double dy = -y;
        double r = rot;

        leftFront.setPower(Range.clip(dy - dx - (r * 0.9), -1, 1) * speedMod);
        rightFront.setPower(Range.clip(-dy - dx - (r * 0.9), -1, 1) * speedMod);
        leftBack.setPower(Range.clip(dy + dx - (r * 0.9), -1, 1) * speedMod);
        rightBack.setPower(Range.clip(-dy + dx - (r * 0.9), -1, 1) * speedMod);

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
            newLeftTarget = leftFront.getCurrentPosition() + (int) ((leftInches * COUNTS_PER_INCH));
            newRightTarget = rightFront.getCurrentPosition() + (int) ((rightInches * COUNTS_PER_INCH));
            leftFront.setTargetPosition(newLeftTarget);
            leftBack.setTargetPosition(newLeftTarget);
            rightFront.setTargetPosition(newRightTarget);
            rightBack.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();

            leftFront.setPower(Math.abs(speed));
            leftBack.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));
            rightFront.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeout) &&
                    (leftFront.isBusy() && rightFront.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftFront.getCurrentPosition(),
                        rightFront.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            StopDriveMotors();

            // Turn off RUN_TO_POSITION
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    void TimeStrafe(double speed, double time){

        Drive(speed, 0, 0, 1);

        WaitFor(time);

        StopDriveMotors();

    }
    void AbsoluteTurn(double speed, double targetAngle){

        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;


        if (currentAngle < targetAngle){

            while (opModeIsActive() && imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < targetAngle) {

                Drive(0, 0, ((currentAngle - targetAngle + 5) / 20), 0.5);
            }


        }else if (currentAngle > targetAngle){

            while (opModeIsActive() && imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > targetAngle) {

                Drive(0, 0, ((targetAngle - currentAngle - 5) / 20), 0.5);
            }
        }

        StopDriveMotors();

    }


    private void ResetDriveEncoders(){


        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void StopDriveMotors(){

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightFront.setPower(0);

    }

    public void ResetEncoders(){

        ResetDriveEncoders();
        latchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        latchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void Sampling(){

        dogevuforia.start();
        WaitFor(.5);

        GoldPosition goldPos = GoldPosition.RIGHT;

        //select gold position

        //check the left and mid, otherwise choose right
        if (dogevuforia.isGold() && dogevuforia.getPosition().y > 125){
            if(dogevuforia.getGoldXPosition() > 100 && dogevuforia.getGoldXPosition() < 250){
                goldPos = GoldPosition.MID;
            }else if (dogevuforia.getGoldXPosition() > 425 && dogevuforia.getGoldXPosition() < 600){
                goldPos = GoldPosition.LEFT;
            }

        }else{

            goldPos = GoldPosition.RIGHT;
        }

        WaitFor(0.5);

        Drive(1, 0, 0, 0.23);
        WaitFor(0.4);

        StopDriveMotors();

        WaitFor(0.25);

      /* while (runTime.seconds() < getNewTime(5)) {
            telemetry.addData("Gold pos:", goldPos);
            telemetry.update();

        }
*/

        if(goldPos == GoldPosition.LEFT){

            Drive(0, -1, 0, 0.23);

        }else{

            Drive(0, 1, 0, 0.23);

        }

        switch (goldPos){

            case LEFT:
                WaitFor(0.4);
                break;

            case MID:
                WaitFor(0.3);
                break;

            case RIGHT:
                WaitFor(0.9);
                break;

        }

        StopDriveMotors();

        WaitFor(0.5);

        Drive(1, 0, 0, 0.5);

        WaitFor(1.2);
        StopDriveMotors();

        /*WaitFor(0.6);

        StopDriveMotors();
        WaitFor(0.25);

        Drive(-1, 0, 0, 0.5);
        WaitFor(0.7);

        StopDriveMotors();
        WaitFor(0.25);

        Drive(0, 1, 0, 0.4);

        switch (goldPos){

            case LEFT:
                WaitFor(1.2);
                break;

            case MID:
                WaitFor(0.7);
                break;

            case RIGHT:
                WaitFor(0.3);
                break;

        }

        StopDriveMotors();*/

        /*switch(goldPos){

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
        }*/

    }

}

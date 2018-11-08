package org.firstinspires.ftc.teamcode.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.HardwareRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.lang.Math;

@TeleOp(name="TeleOp Go di di go dee go", group="Linear Opmode")
public class MainTeleOp extends OpMode {


    private enum DriveSpeed{

        SLOW, FAST

    }

    private enum LiftPosition{


    }

    private enum ArmExtensionPosition{


    }

    private enum ArmRotationPosition{

    }




    //fields to control speed of drive train
    private DriveSpeed driveSpeed;
    private double speedMod;

    //fields to control the postiion of the grabber
    private LiftPosition liftPosition;
    private ArmExtensionPosition armExtensionPosition;
    private ArmRotationPosition armRotationPosition;


    HardwareRobot robot;


    @Override
    public void init(){

        robot = new HardwareRobot(hardwareMap);
    }

    @Override
    public void loop(){

        Telemetry();

        DriveControl();

        LiftControl();

        ArmRotationControl();

        ArmExtensionControl();




    }

    private void Telemetry(){

    //display run data here

    }

    //sets power to motors from joystick input based on mecanum setup
    public void DriveControl(){

        CheckSpeed();

        //Reverse the y coordinate
        double x1 = gamepad1.left_stick_x;
        double y1 = -gamepad1.left_stick_y;
        float x2 = gamepad1.right_stick_x;

        //Left joystick controls translation, right joystick controls turning
        robot.frontLeft.setPower(clamp(speedMod*(y1 + x1 + x2), -1, 1));
        robot.frontRight.setPower(clamp(speedMod*(y1 - x1 - x2), -1, 1));
        robot.backLeft.setPower(clamp(speedMod*(y1 - x1 + x2), -1, 1));
        robot.backRight.setPower(clamp(speedMod*(y1 + x1 -x2), -1, 1));


        // trig implementation for mecanum drive (unused alternative)
        /**
        double angle = Math.atan2(y1, x1);

        robot.frontLeft.setPower(Math.sin(-angle + Math.PI/4) - x2);
        robot.frontRight.setPower(Math.cos(-angle + Math.PI/4) + x2);
        robot.backLeft.setPower(Math.cos(-angle + Math.PI/4) - x2);
        robot.backRight.setPower(Math.sin(-angle + Math.PI/4) + x2);

        **/


    }


    //Check first controller d-pad for speed modifier
    public void CheckSpeed(){

        if (gamepad1.left_bumper && driveSpeed == DriveSpeed.FAST) {

            driveSpeed = DriveSpeed.SLOW;

        } else if (gamepad1.left_bumper && driveSpeed == DriveSpeed.SLOW) {

            driveSpeed = DriveSpeed.FAST;
        }

        switch (driveSpeed){

            case SLOW: speedMod = .4;
                        break;

            case FAST: speedMod = .75;
                        break;
        }


    }

    //constants for encoder values


    //Second controller joysticks (left for linear, right for rotation)
    public void LiftControl() {

        final int MAX_POSITION = 3750;
        final int MID_POSITION = 1750;
        final int MIN_POSITION = 250;


    }

    public void ArmRotationControl(){

        final int MAX_POSITION = 3750;
        final int MID_POSITION = 1750;
        final int MIN_POSITION = 250;

        if (gamepad2.right_stick_y != 0) {

            if (robot.rotationMotor.getCurrentPosition() > MIN_POSITION && robot.rotationMotor.getCurrentPosition() < MAX_POSITION) {

                robot.rotationMotor.setPower(-gamepad2.right_stick_y * 0.6);

            } else {

                robot.rotationMotor.setPower(0);
            }

        }

    }

    public void ArmExtensionControl(){

        final int MAX_POSITION = 3750;
        final int MID_POSITION = 1750;
        final int MIN_POSITION = 250;

        if (gamepad2.left_stick_y != 0) {

            if (robot.extensionMotor.getCurrentPosition() > MIN_POSITION && robot.extensionMotor.getCurrentPosition() < MAX_POSITION) {

                robot.extensionMotor.setPower(-gamepad2.left_stick_y * 0.6);

            } else {

                robot.extensionMotor.setPower(0);
            }

        }
    }

    //left and right triggers for intake/outtake, else stop collection motor
    private void GrabControl(){

        if (gamepad2.left_trigger != 0){
            robot.collectionMotor.setPower(-gamepad2.left_trigger * .7);

        } else if(gamepad2.right_trigger != 0){

            robot.collectionMotor.setPower(gamepad2.left_trigger * .7);

        } else {

            robot.collectionMotor.setPower(0);

        }

    }

    //stop all motors
    public void stop(){

        robot.StopAllMotors();

    }


    //clamps values outside of the range to the min/max
    private static double clamp(double val, double min, double max) {

        return Math.max(min, Math.min(max, val));
    }
}

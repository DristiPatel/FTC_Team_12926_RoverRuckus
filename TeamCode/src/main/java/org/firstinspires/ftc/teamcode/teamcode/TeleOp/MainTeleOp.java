package org.firstinspires.ftc.teamcode.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.teamcode.Testers.MecanumTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.lang.Math;

@TeleOp(name="Main TeleOop", group="Linear Opmode")
public class MainTeleOp extends OpMode {


    private enum DriveSpeed{

        SLOWEST, SLOW, FAST

    }

    //unused, here if we want to set certain encoder positions for the lift and arm
    private enum LiftPosition{
    }

    private enum ArmExtensionPosition{
    }

    private enum ArmRotationPosition{

        LOWERED, RAISED, SUCKER, TO_LOWERED, TO_RAISED, TO_SUCKER, STATIC, MANUAL

    }

    //fields to control the postiion of the grabber, also unused
    private LiftPosition liftPosition;
    private ArmExtensionPosition armExtensionPosition;

    private ArmRotationPosition armRotationPosition;
    private int manualPosition;



    //fields to control speed of drive train
    private DriveSpeed driveSpeed;
    private double speedMod;


    //declare the robot!
    HardwareRobot robot;


    @Override
    public void init(){

        //map the robot hardware
        robot = new HardwareRobot(hardwareMap);

        //set the drive speed to default
        driveSpeed = DriveSpeed.FAST;
        speedMod = 1;

        //robot.rotationMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.rotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armRotationPosition = ArmRotationPosition.LOWERED;
        manualPosition = 0;
    }

    @Override
    public void loop(){

        DriveControl();

        //LiftControl();

        //ArmRotationControl();

        //ArmExtensionControl();

        //GrabControl();

        telemetry.addData("Speed:  ", speedMod);
        //telemetry.addData("Lift Position", robot.liftMotor.getCurrentPosition());
        //telemetry.addData("Extension Position", robot.extensionMotor.getCurrentPosition());
        //telemetry.addData("Rotation Position", robot.rotationMotor.getCurrentPosition());
        //telemetry.addData("Rotation Position name  ", armRotationPosition);
        //telemetry.addData("target position: ", robot.rotationMotor.getTargetPosition());
        telemetry.update();


    }



    //sets power to motors from joystick input based on mecanum setup
    public void DriveControl(){

        //sets the speedMod by checking for speed adjustments via d-pad
        CheckSpeed();




    }


    //Check first controller d-pad for speed modifier
    public void CheckSpeed(){

        if (gamepad1.dpad_down) {

            driveSpeed = DriveSpeed.SLOW;

        } else if (gamepad1.dpad_up ) {

            driveSpeed = DriveSpeed.FAST;

        }else if (gamepad1.dpad_left ){

            driveSpeed = DriveSpeed.SLOWEST;

        }

        //set the speed multiplier based on enum
        switch (driveSpeed){

            case SLOW: speedMod = .5;
                        break;

            case FAST: speedMod = 1;
                        break;

            case SLOWEST: speedMod = .25;

                        break;
        }


    }

    //constants for encoder values


    //Second controller joysticks (left for linear, right for rotation)
    public void LiftControl() {

        final int MAX_POSITION = 3750;
        final int MIN_POSITION = 0;

        if (gamepad2.dpad_up ){

            robot.liftMotor.setPower(-.2);


        }else if (gamepad2.dpad_down){

            robot.liftMotor.setPower(.2);
        }else{

            robot.liftMotor.setPower(0);
        }

    }

    public void CheckArmRotationControl() {


        if (gamepad2.b) {

            armRotationPosition = ArmRotationPosition.TO_LOWERED;

        } else if (gamepad2.y) {

            armRotationPosition = ArmRotationPosition.TO_RAISED;

        } else if (gamepad2.x) {

            armRotationPosition = ArmRotationPosition.TO_SUCKER;

        }else if (gamepad2.right_stick_y != 0){

            armRotationPosition = ArmRotationPosition.MANUAL;

        }

        if(!gamepad2.b && armRotationPosition == ArmRotationPosition.TO_LOWERED){

            armRotationPosition = ArmRotationPosition.LOWERED;

        }else if(!gamepad2.y && armRotationPosition == ArmRotationPosition.TO_RAISED){

            armRotationPosition = ArmRotationPosition.RAISED;

        }else if(!gamepad2.x && armRotationPosition == ArmRotationPosition.TO_SUCKER){

            armRotationPosition = ArmRotationPosition.SUCKER;
        }

    }


    public void ArmRotationControl(){

        final int SUCKER_POSITION = 390;
        final int MAX_POSITION = 130;
        final int MIN_POSITION = 20;
        final double power = .8;


        CheckArmRotationControl();

        switch(armRotationPosition){

            case LOWERED:

                if (robot.rotationMotor.getCurrentPosition() > 20) {
                    robot.rotationMotor.setTargetPosition(MIN_POSITION);
                    robot.rotationMotor.setPower(power);

                }else{

                    armRotationPosition = ArmRotationPosition.STATIC;
                }
                manualPosition = MIN_POSITION;

               break;

            case RAISED:


                    robot.rotationMotor.setTargetPosition(MAX_POSITION);
                    robot.rotationMotor.setPower(power);
                    manualPosition = MAX_POSITION;
                    
                break;

            case SUCKER:

                //if(robot.rotationMotor.getCurrentPosition() < SUCKER_POSITION) {
                    robot.rotationMotor.setTargetPosition(SUCKER_POSITION);
                    robot.rotationMotor.setPower(power);
                    manualPosition = SUCKER_POSITION;
                //}else{

                  //  armRotationPosition = ArmRotationPosition.STATIC;
                //}
                break;

            case STATIC:
                robot.rotationMotor.setPower(0);
                break;

            case MANUAL:

                if (gamepad2.dpad_up || gamepad2.dpad_down){

                    if(gamepad2.dpad_down) {
                        manualPosition +=2;
                        robot.rotationMotor.setTargetPosition(manualPosition);
                        robot.rotationMotor.setPower(power);
                    }else {
                        manualPosition-=2;
                        robot.rotationMotor.setTargetPosition(manualPosition);
                        robot.rotationMotor.setPower(power);

                    }
                }else{

                    robot.rotationMotor.setTargetPosition(manualPosition);
                    robot.rotationMotor.setPower(power);
                }
                break;

        }

    }

    public void ArmExtensionControl(){

        final int MAX_POSITION = 1100;
        final int MIN_POSITION = -100;
        final double power = .6;

        if (gamepad2.left_stick_y < 0 ) {


            robot.extensionMotor.setTargetPosition(MAX_POSITION);
            robot.extensionMotor.setPower(-gamepad2.left_stick_y * power);

        }else if(gamepad2.left_stick_y > 0 ) {


            robot.extensionMotor.setTargetPosition(MIN_POSITION);
            robot.extensionMotor.setPower(-gamepad2.left_stick_y * power);

        }else{

            robot.extensionMotor.setPower(0);
        }




    }

    //left and right triggers for intake/outtake, else stop collection motor
    private void GrabControl(){

        if (gamepad2.left_trigger != 0){
            robot.collectionMotor.setPower(-gamepad2.left_trigger * .7);

        }
        if(gamepad2.right_trigger != 0){

            robot.collectionMotor.setPower(gamepad2.right_trigger * .7);

        }
        if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0){

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

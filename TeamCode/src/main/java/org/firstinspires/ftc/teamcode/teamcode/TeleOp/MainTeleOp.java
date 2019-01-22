package org.firstinspires.ftc.teamcode.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.teamcode.Testers.MecanumTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;

@TeleOp(name="Main TeleOop", group="Linear Opmode")
public class MainTeleOp extends OpMode {


    private enum DriveSpeed{

        SLOWEST, SLOW, FAST

    }


    //fields to control speed of drive train
    private DriveSpeed driveSpeed;
    private double speedMod;
    private ElapsedTime runTime;

    private boolean locked;
    private int target;

    //declare the robot!
    HardwareRobot robot;


    @Override
    public void init(){

        //map the robot hardware
        robot = new HardwareRobot(hardwareMap);

        //set the drive speed to default
        driveSpeed = DriveSpeed.FAST;
        speedMod = 1;
        locked = false;
        target = 0;

        runTime = new ElapsedTime();
        //robot.ResetAllEncoders(); uwu



    }

    @Override
    public void loop(){

        DriveControl();

        LiftControl();

        ScoopControl();

        ExtensionControl();

        //TiltControl();

        GrabControl();

        telemetry.addData("Speed:  ", speedMod);
        telemetry.addData("Lift Position", robot.liftMotor.getCurrentPosition());
        telemetry.addData("Extension Position", robot.extensionMotor.getCurrentPosition());
        telemetry.addData("Target Pos", robot.extensionMotor.getTargetPosition());

        telemetry.update();

    }

    //omni wheel setup with strafe
    public void DriveControl(){

        //sets the speedMod by checking for speed adjustments via d-pad
        CheckSpeed();

        robot.leftDrive.setPower(speedMod*(-gamepad1.left_stick_y + gamepad1.right_stick_x));
        robot.rightDrive.setPower(speedMod*(-gamepad1.left_stick_y - gamepad1.right_stick_x));

        robot.strafeDrive.setPower(speedMod*(gamepad1.left_stick_x));


    }


    //Check first controller d-pad for speed modifier
    public void CheckSpeed(){

        if (gamepad1.x) {

            driveSpeed = DriveSpeed.SLOW;

        } else if (gamepad1.y) {

            driveSpeed = DriveSpeed.FAST;

        }

        //set the speed multiplier based on enum
        switch (driveSpeed){

            case SLOW: speedMod = .5;
                        break;

            case FAST: speedMod = 1;
                        break;

        }


    }

    //Second controller joysticks (left for linear, right for rotation)
    public void LiftControl() {

        if (gamepad1.dpad_down){

            robot.liftMotor.setPower(.7);


        }else if (gamepad1.dpad_up){

            robot.liftMotor.setPower(-.7);

        }else{

            robot.liftMotor.setPower(0);
        }

    }

    public void ScoopControl() {

        if (gamepad2.a) {


            robot.scoopServo.setPosition(Servo.MIN_POSITION);
            robot.swingServo1.setPosition(.1);
            robot.swingServo2.setPosition(.9);

            double waitTime = runTime.seconds() + 1.1;
            while (getRuntime() < waitTime) {

                //delay
            }

            robot.scoopServo.setPosition(.5);
            telemetry.addData("arm pos: ", "Lowered");

        }

        if (gamepad2.y){

            double waitTime;

            robot.scoopServo.setPosition(Servo.MIN_POSITION);

            waitTime = runTime.seconds() + .5;

            while (getRuntime() < waitTime) {

                //delay
            }

            robot.swingServo1.setPosition(.7);
            robot.swingServo2.setPosition(.3);

            //extend outwards for momentum

            waitTime = runTime.seconds() + .05;
            while (getRuntime() < waitTime) {

                //delay
            }

            robot.scoopServo.setPosition(Servo.MAX_POSITION);

            waitTime = runTime.seconds() + .5;
            while (getRuntime() < waitTime) {

                //delay
            }


            robot.swingServo1.setPosition(Servo.MAX_POSITION);
            robot.swingServo2.setPosition(Servo.MIN_POSITION);

            waitTime = runTime.seconds() + .1;
            while (getRuntime() < waitTime) {

             //delay
            }

            robot.scoopServo.setPosition(.5);

            telemetry.addData("arm pos: ", "Near");

        }

        if (gamepad2.x){

            double waitTime;

            robot.scoopServo.setPosition(Servo.MIN_POSITION);

            waitTime = runTime.seconds() + .5;

            while (getRuntime() < waitTime) {

                //delay
            }

            robot.swingServo1.setPosition(.7);
            robot.swingServo2.setPosition(.3);

            //extend outwards for momentum

            waitTime = runTime.seconds() + .05;
            while (getRuntime() < waitTime) {

                //delay
            }

            robot.scoopServo.setPosition(Servo.MAX_POSITION);

            waitTime = runTime.seconds() + .5;
            while (getRuntime() < waitTime) {

                //delay
            }


            robot.swingServo1.setPosition(.8);
            robot.swingServo2.setPosition(.2);

            waitTime = runTime.seconds() + .1;
            while (getRuntime() < waitTime) {

                //delay
            }

            robot.scoopServo.setPosition(Servo.MAX_POSITION);

            waitTime = runTime.seconds() + .2;
            while (getRuntime() < waitTime) {

                //delay
            }

            robot.scoopServo.setPosition(.5);

            robot.swingServo1.setPosition(Servo.MAX_POSITION);
            robot.swingServo2.setPosition(Servo.MIN_POSITION);

            telemetry.addData("arm pos: ", "Far");


        }
    }

    //to be done
    public void ExtensionControl(){

        final int MAX_POSITION = 230;
        final int MIN_POSITION = -100;
        final double power = .5;


        robot.extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (gamepad2.left_stick_y < 0 ) {

            //robot.extensionMotor.setTargetPosition(robot.extensionMotor.getCurrentPosition() + (int)(gamepad2.left_stick_y * 15));
            robot.extensionMotor.setPower( gamepad2.left_stick_y *power);

        }else if (gamepad2.left_stick_y > 0){

            //robot.extensionMotor.setTargetPosition(robot.extensionMotor.getCurrentPosition() + (int)(gamepad2.left_stick_y * 15));
            robot.extensionMotor.setPower(gamepad2.left_stick_y *power);

        }else if (gamepad2.left_stick_y == 0 && !gamepad2.right_bumper){

            robot.extensionMotor.setPower(0);
        }

        if(!locked && gamepad2.right_bumper)
            target = robot.extensionMotor.getCurrentPosition();

        if (gamepad2.right_bumper){

            locked = true;

            robot.extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.extensionMotor.setTargetPosition(target);
            robot.extensionMotor.setPower(1);

        }else{

            locked = false;
        }


    }

/*
    private void TiltControl(){

        final double power = .5;

        if (gamepad2.dpad_up){

            robot.rotationMotor.setPower(power);

        }else if (gamepad2.dpad_down){

            robot.rotationMotor.setPower(-power);

        }else{

            robot.rotationMotor.setPower(0);
        }


    }
*/

    //left and right triggers for intake/outtake, else stop collection motor
    private void GrabControl(){

        if (gamepad2.left_trigger != 0){
            robot.collectionMotor.setPower(gamepad2.left_trigger * .7);

        }
        if(gamepad2.right_trigger != 0){

            robot.collectionMotor.setPower(-gamepad2.right_trigger * .7);

        }

        if (gamepad2.left_bumper){


            robot.collectionMotor.setPower(.3);

        }

        if (gamepad2.right_bumper){

            robot.collectionMotor.setPower(-.3);

        }

        if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0 && !gamepad2.left_bumper && !gamepad2.right_bumper){

            robot.collectionMotor.setPower(0);

        }



    }



}

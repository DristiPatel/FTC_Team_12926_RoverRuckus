package org.firstinspires.ftc.teamcode.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.BetterHardwareRobot;
import org.firstinspires.ftc.teamcode.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.teamcode.Testers.MecanumTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;

@TeleOp(name="Main TeleOop", group="Linear Opmode")
@Disabled
public class MainTeleOp extends OpMode {


    private enum DriveSpeed{

        SLOWEST, SLOW, FAST

    }


    //fields to control speed of drive train
    private DriveSpeed driveSpeed;
    private double speedMod;
    private ElapsedTime runTime;


    boolean limitsOn = true;

    //declare the robot!
    BetterHardwareRobot robot;


    @Override
    public void init(){

        //map the robot hardware
        robot = new BetterHardwareRobot(hardwareMap);

        //set the drive speed to default
        driveSpeed = DriveSpeed.FAST;
        speedMod = 1;


        runTime = new ElapsedTime();



    }

    @Override
    public void loop(){

        setLimits();

        DriveControl();

        if(limitsOn) {

            LiftControl();

            ExtensionControl();

            TiltControl();

            ScoopMotorControl();

            Retract();

         //limits off
        }else{

            LiftControl2();

            ExtensionControl2();

            TiltControl2();

            ScoopMotorControl2();
        }

        GrabControl();
        ScoopServoControl();




        telemetry.addData("Speed:  ", speedMod);
        telemetry.addData("Lift Position", robot.liftMotor.getCurrentPosition());
        telemetry.addData("Extension Position", robot.extensionMotor.getCurrentPosition());
        telemetry.addData("Scoop Position", robot.scoopMotor.getCurrentPosition());
        telemetry.addData("Tilt Position", robot.flipMotor.getCurrentPosition());
        telemetry.addData("Limits On", limitsOn);
        telemetry.addData("lift speed", robot.liftMotor.getPower());

        //telemetry.addData("bucket position", robot.scoopServo.getPosition());

        telemetry.update();

    }

    public void setLimits(){

        if (gamepad1.start && gamepad1.dpad_up){

            limitsOn = false;
        }

        if (gamepad1.start && gamepad1.dpad_down){

            limitsOn = true;
        }
    }

    //omni wheel setup with strafe
    public void DriveControl(){

        //sets the speedMod by checking for speed adjustments via d-pad
        CheckSpeed();


        telemetry.addData("Speed:  ", speedMod);
        telemetry.update();

        //y is negative in the up direction and positive in the down direction, hence the reverse
        double x1 = gamepad1.left_stick_x;
        double y1 = -gamepad1.left_stick_y;
        double x2 = gamepad1.right_stick_x;

        //trig implementation
        double power = Math.hypot(x1, y1);
        double angle = Math.atan2(y1, x1) - Math.PI/4;

        robot.frontLeft.setPower(speedMod*(power * Math.cos(angle) + x2));
        robot.frontRight.setPower(speedMod*(power * Math.sin(angle) - x2));
        robot.backLeft.setPower(speedMod*(power * Math.sin(angle) + x2));
        robot.backRight.setPower(speedMod*(power * Math.cos(angle) - x2));

    }


    //Check first controller d-pad for speed modifier
    public void CheckSpeed(){

        if (gamepad1.dpad_left) {

            driveSpeed = DriveSpeed.SLOW;

        } else if (gamepad1.dpad_right) {

            driveSpeed = DriveSpeed.FAST;

        }

        //set the speed multiplier based on enum
        switch (driveSpeed){

            case SLOW: speedMod = .65;
                        break;

            case FAST: speedMod = 1;
                        break;

        }
    }

    //first controller dpad for lift
    public void LiftControl() {


        double MIN_POSITION = -6784;
        double MAX_POSITION = 0;


        if (gamepad1.left_bumper){

            robot.liftMotor.setPower(-.7);


        }else if (gamepad1.right_bumper){

            robot.liftMotor.setPower(.7);

        }else{

            robot.liftMotor.setPower(0);
        }

    }

    public void ScoopMotorControl() {


        final int MAX_POSITION = 1100;
        final int MIN_POSITION = 0;
        final double power = 1;

        //add encoder limits
        if (gamepad1.left_trigger > 0 && robot.scoopMotor.getCurrentPosition() > MIN_POSITION){

            robot.scoopMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.scoopMotor.setPower(-1);

        }else if(gamepad1.right_trigger > 0 && robot.scoopMotor.getCurrentPosition() < MAX_POSITION){

            robot.scoopMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.scoopMotor.setPower(1);
        }

        else if (gamepad1.b && robot.scoopMotor.getCurrentPosition() > MIN_POSITION){

            robot.scoopMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.scoopMotor.setTargetPosition(MIN_POSITION);
            robot.scoopMotor.setPower(-1);
/*
        }else if (gamepad1.a && robot.scoopMotor.getCurrentPosition() < MAX_POSITION){

            robot.scoopMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.scoopMotor.setTargetPosition(MAX_POSITION);
            robot.scoopMotor.setPower(1);
 */
        }else{

            if(robot.scoopMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {

                robot.scoopMotor.setPower(0);
            }
        }


    }

    public void ScoopServoControl(){


       if (gamepad1.x){
           robot.scoopServo.setPosition(Servo.MAX_POSITION);

        }

        if (gamepad1.y){

            ElapsedTime delayTime = new ElapsedTime();

            robot.scoopServo.setPosition(Servo.MIN_POSITION);
/*
            while (robot.scoopServo.getPosition() > 0.001){

                robot.scoopServo.setPosition(robot.scoopServo.getPosition() - .02);

                telemetry.addData("servo pos ", robot.scoopServo.getPosition());
                delayTime.reset();

                while (delayTime.seconds() < .02){
                    //idle
                }

            }
            */
        }


    }

    //second controller joystick for extension
    public void ExtensionControl(){

        final int MAX_POSITION = 911;
        final int MIN_POSITION = 0;
        final double power = 1;

        //add encoder limits
        if (gamepad2.left_stick_y < 0 && robot.extensionMotor.getCurrentPosition() < MAX_POSITION){

            robot.extensionMotor.setPower(-power * gamepad2.left_stick_y);

        }else if(gamepad2.left_stick_y > 0 && robot.extensionMotor.getCurrentPosition() > MIN_POSITION){

            robot.extensionMotor.setPower(-power * gamepad2.left_stick_y);
        }else{

            robot.extensionMotor.setPower(0);
        }

    }

    public void TiltControl(){

        final double MAX_POSITION = 264;
        final double MIN_POSITION = 0;

        final double power = .75;

        if (-gamepad2.right_stick_y > 0 ){

            robot.flipMotor.setPower(power * -gamepad2.right_stick_y);

        }else if(-gamepad2.right_stick_y < 0){

            robot.flipMotor.setPower(power * -gamepad2.right_stick_y);
        }else{

            robot.flipMotor.setPower(0);
        }

    }


    //left and right triggers for intake/outtake, else stop  servo
    private void GrabControl(){

        final double power = 1;

        if (gamepad2.left_trigger > 0){

            robot.intakeServo.setPower(power * gamepad2.left_trigger);

        }else if (gamepad2.right_trigger > 0){

            robot.intakeServo.setPower(-power * gamepad2.right_trigger);
        }else{

            robot.intakeServo.setPower(0);
        }
    }

    private void Retract(){

        if (gamepad2.a){


            robot.scoopMotor.setPower(-1);
            robot.extensionMotor.setPower(-1);

            while (robot.scoopMotor.getCurrentPosition() > 10){

            }
            robot.scoopMotor.setPower(0);

            while (robot.extensionMotor.getCurrentPosition() > 10){

            }
            robot.extensionMotor.setPower(0);

            while (robot.flipMotor.getCurrentPosition() > 5){

                robot.flipMotor.setPower(-.5);
            }
            robot.flipMotor.setPower(0);

        }

    }

    //LIMITS OFF

    public void ScoopMotorControl2() {


        final int MAX_POSITION = 1100;
        final int MIN_POSITION = 0;
        final double power = 1;

        //add encoder limits
        if (gamepad1.left_trigger > 0) {

            robot.scoopMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.scoopMotor.setPower(power * -gamepad1.left_trigger);

        }else if(gamepad1.right_trigger > 0 ){

            robot.scoopMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.scoopMotor.setPower(power * gamepad1.right_trigger);
        }
/*
        else if (gamepad1.b && robot.scoopMotor.getCurrentPosition() > MIN_POSITION){

            robot.scoopMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.scoopMotor.setTargetPosition(MIN_POSITION);
            robot.scoopMotor.setPower(-1);

        }else if (gamepad1.a && robot.scoopMotor.getCurrentPosition() < MAX_POSITION){

            robot.scoopMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.scoopMotor.setTargetPosition(MAX_POSITION);
            robot.scoopMotor.setPower(1);

           } */
        else{

            if(robot.scoopMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {

                robot.scoopMotor.setPower(0);
            }
        }


    }

    public void TiltControl2(){


        final double power = .75;

        if (-gamepad2.right_stick_y > 0 ){

            robot.flipMotor.setPower(power * -gamepad2.right_stick_y);

        }else if(-gamepad2.right_stick_y < 0 ){

            robot.flipMotor.setPower(power * -gamepad2.right_stick_y);
        }else{

            robot.flipMotor.setPower(0);
        }

    }

    public void LiftControl2() {


        if (gamepad1.left_bumper){

            robot.liftMotor.setPower(-.7);


        }else if (gamepad1.right_bumper){

            robot.liftMotor.setPower(.7);

        }else{

            robot.liftMotor.setPower(0);
        }

    }

    public void ExtensionControl2(){

        final double power = 1;

        //add encoder limits
        if (gamepad2.left_stick_y < 0 ){

            robot.extensionMotor.setPower(-power * gamepad2.left_stick_y);

        }else if(gamepad2.left_stick_y > 0 ){

            robot.extensionMotor.setPower(-power * gamepad2.left_stick_y);
        }else{

            robot.extensionMotor.setPower(0);
        }

    }



}

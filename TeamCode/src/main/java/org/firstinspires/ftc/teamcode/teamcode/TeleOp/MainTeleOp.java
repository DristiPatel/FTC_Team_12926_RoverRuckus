package org.firstinspires.ftc.teamcode.teamcode.TeleOp;
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
    BetterHardwareRobot robot;


    @Override
    public void init(){

        //map the robot hardware
        robot = new BetterHardwareRobot(hardwareMap);

        //set the drive speed to default
        driveSpeed = DriveSpeed.FAST;
        speedMod = 1;
        locked = false;
        target = 0;

        runTime = new ElapsedTime();



    }

    @Override
    public void loop(){

        DriveControl();

        LiftControl();

        ExtensionControl();

        TiltControl();

        GrabControl();

        ScoopMotorControl();

        ScoopServoControl();


        telemetry.addData("Speed:  ", speedMod);
        telemetry.addData("Lift Position", robot.liftMotor.getCurrentPosition());
        //telemetry.addData("Extension Position", robot.extensionMotor.getCurrentPosition());
        telemetry.addData("Scoop Position", robot.scoopMotor.getCurrentPosition());

        //telemetry.addData("bucket position", robot.scoopServo.getPosition());

        telemetry.update();

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

        if (gamepad1.a) {

            driveSpeed = DriveSpeed.SLOW;

        } else if (gamepad1.b) {

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

        //lift max: 17000
        //lift min:

        if (gamepad1.dpad_down){

            robot.liftMotor.setPower(-.7);


        }else if (gamepad1.dpad_up){

            robot.liftMotor.setPower(.7);

        }else{

            robot.liftMotor.setPower(0);
        }

    }

    public void ScoopMotorControl() {


        final int MAX_POSITION = 230;
        final int MIN_POSITION = -100;
        final double power = 1;

        //add encoder limits
        if (gamepad1.left_trigger > 0){

            robot.scoopMotor.setPower(power * -gamepad1.left_trigger);

        }else if(gamepad1.right_trigger > 0){

            robot.scoopMotor.setPower(power * gamepad1.right_trigger);
        }else{

            robot.scoopMotor.setPower(0);
        }

    }

    public void ScoopServoControl(){


       if (gamepad1.x){
           robot.scoopServo.setPosition(Servo.MAX_POSITION);

        }

        if (gamepad1.y){

            ElapsedTime delayTime = new ElapsedTime();

            robot.scoopServo.setPosition(Servo.MAX_POSITION);

            while (robot.scoopServo.getPosition() > 0.001){

                robot.scoopServo.setPosition(robot.scoopServo.getPosition() - .02);

                telemetry.addData("servo pos ", robot.scoopServo.getPosition());
                delayTime.reset();

                while (delayTime.seconds() < .02){
                    //idle
                }

            }
        }


    }

    //second controller joystick for extension
    public void ExtensionControl(){

        final int MAX_POSITION = 230;
        final int MIN_POSITION = -100;
        final double power = 1;

        //add encoder limits
        if (-gamepad2.left_stick_y > 0){

            robot.extensionMotor.setPower(power * -gamepad2.left_stick_y);

        }else if(-gamepad2.left_stick_y < 0){

            robot.extensionMotor.setPower(power * -gamepad2.left_stick_y);
        }else{

            robot.extensionMotor.setPower(0);
        }

    }

    public void TiltControl(){

        final double power = .5;

        if (-gamepad2.right_stick_y > 0){

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

        }else if (gamepad2.right_trigger < 0){

            robot.intakeServo.setPower(-power * gamepad2.right_trigger);
        }else{

            robot.intakeServo.setPower(0);
        }
    }

}

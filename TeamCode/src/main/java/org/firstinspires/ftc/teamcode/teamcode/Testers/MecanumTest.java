package org.firstinspires.ftc.teamcode.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.teamcode.Autonomous.BetterAutonomousRobot;
import org.firstinspires.ftc.teamcode.teamcode.BetterHardwareRobot;
import org.firstinspires.ftc.teamcode.teamcode.HardwareRobot;


@TeleOp(name="camera servo tester", group="Linear Opmode")
public class MecanumTest extends LinearOpMode {


    private enum DriveSpeed{

        SLOW, FAST

    }

    private DcMotor frontLeft, frontRight, backLeft, backRight;


    private DriveSpeed driveSpeed;
    private double speedMod;
    BetterHardwareRobot robot;


    @Override
    public void runOpMode(){

        //initialize motors

        robot = new BetterHardwareRobot(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.x){

                robot.webcamServo.setPosition(Servo.MAX_POSITION);
            }

            if(gamepad1.y){

                robot.webcamServo.setPosition(Servo.MIN_POSITION);
            }

        }

    }




}

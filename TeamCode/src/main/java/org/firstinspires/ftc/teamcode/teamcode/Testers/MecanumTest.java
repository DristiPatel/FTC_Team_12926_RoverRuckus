package org.firstinspires.ftc.teamcode.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@TeleOp(name="Mecanum Test", group="Linear Opmode")
public class MecanumTest extends LinearOpMode {


    private enum DriveSpeed{

        SLOW, FAST

    }

    private DcMotor frontLeft, frontRight, backLeft, backRight;


    private DriveSpeed driveSpeed;
    private double speedMod;



    @Override
    public void runOpMode(){

        //initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        frontRight = hardwareMap.get(DcMotor.class, "Front Right");
        backLeft = hardwareMap.get(DcMotor.class, "Back Left");
        backRight = hardwareMap.get(DcMotor.class, "Back Right");

        //reverse a side of motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        driveSpeed = DriveSpeed.FAST;
        speedMod = 1;

        waitForStart();

        while(opModeIsActive()) {

          CheckSpeed();


            telemetry.addData("Speed:  ", speedMod);
            telemetry.update();

            //y is negative in the up direction and positive in the down direction, hence the reverse
            double x1 = gamepad1.left_stick_x;
            double y1 = -gamepad1.left_stick_y;
            double x2 = -gamepad1.right_stick_x;

           //trig implementation
            double power = Math.hypot(x1, y1);
            double angle = Math.atan2(y1, x1) - Math.PI/4;

            frontLeft.setPower(speedMod*(power * Math.cos(angle) + x2));
            frontRight.setPower(speedMod*(power * Math.sin(angle) - x2));
            backLeft.setPower(speedMod*(power * Math.sin(angle) + x2));
            backRight.setPower(speedMod*(power * Math.cos(angle) - x2));



        }

    }

    public void CheckSpeed(){

        if (gamepad1.left_bumper && driveSpeed == DriveSpeed.FAST) {

            driveSpeed = DriveSpeed.SLOW;

        } else if (gamepad1.right_bumper && driveSpeed == DriveSpeed.SLOW) {

            driveSpeed = DriveSpeed.FAST;
        }

        switch (driveSpeed){

            case SLOW: speedMod = .5;
                break;

            case FAST: speedMod = 1;
                break;
        }


    }


}

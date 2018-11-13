package org.firstinspires.ftc.teamcode.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


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



        frontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        frontRight = hardwareMap.get(DcMotor.class, "Front Right");
        backLeft = hardwareMap.get(DcMotor.class, "Back Left");
        backRight = hardwareMap.get(DcMotor.class, "Back Right");


        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        speedMod = .6;
        driveSpeed = DriveSpeed.FAST;

        waitForStart();

        while(opModeIsActive()) {

            CheckSpeed();

            double x1 = gamepad1.left_stick_x;
            double y1 = -gamepad1.left_stick_y;
            double x2 = gamepad1.right_stick_x;

            //Left joystick controls translation, right joystick controls turning
            frontLeft.setPower(Range.clip(y1 + x1 + x2, -1, 1));
            frontRight.setPower(Range.clip(y1 - x1 - x2, -1, 1));
            backLeft.setPower(Range.clip(y1 - x1 + x2, -1, 1));
            backRight.setPower(Range.clip(y1 + x1 - x2, -1, 1));


            //trig implementation
            double power = Math.hypot(x1, y1);
            double angle = Math.atan2(y1, x1) - Math.PI/4;

            frontLeft.setPower(power * Math.cos(angle) + x2);
            frontRight.setPower(power * Math.sin(angle) - x2);
            backLeft.setPower(power * Math.sin(angle) + x2);
            backRight.setPower(power * Math.cos(angle) - x2);



        }



    }

    public void CheckSpeed(){

        if (gamepad1.left_bumper && driveSpeed == DriveSpeed.FAST) {

            driveSpeed = DriveSpeed.SLOW;

        } else if (gamepad1.right_bumper && driveSpeed == DriveSpeed.SLOW) {

            driveSpeed = DriveSpeed.FAST;
        }

        switch (driveSpeed){

            case SLOW: speedMod = .4;
                break;

            case FAST: speedMod = .75;
                break;
        }


    }

    private static double clamp(double val, double min, double max) {

        return Math.max(min, Math.min(max, val));
    }
}

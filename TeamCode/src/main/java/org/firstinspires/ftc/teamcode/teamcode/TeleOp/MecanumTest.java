package org.firstinspires.ftc.teamcode.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name="Mecanum Test", group="Linear Opmode")
public class MecanumTest extends OpMode {


    private enum DriveSpeed{

        SLOW, FAST

    }

    private DcMotor frontLeft, frontRight, backLeft, backRight = null;


    private DriveSpeed driveSpeed;
    private double speedMod;


    public void init(){

        frontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        frontRight = hardwareMap.get(DcMotor.class, "Front Right");
        backLeft = hardwareMap.get(DcMotor.class, "Back Left");
        backRight = hardwareMap.get(DcMotor.class, "Back Right");


    }

    public void loop(){

        CheckSpeed();

        double x1 = gamepad1.left_stick_x;
        double y1 = -gamepad1.left_stick_y;
        float x2 = gamepad1.right_stick_x;

        //Left joystick controls translation, right joystick controls turning
        frontLeft.setPower(clamp(speedMod*(y1 + x1 + x2), -1, 1));
        frontRight.setPower(clamp(speedMod*(y1 - x1 - x2), -1, 1));
        backLeft.setPower(clamp(speedMod*(y1 - x1 + x2), -1, 1));
        backRight.setPower(clamp(speedMod*(y1 + x1 -x2), -1, 1));

        /**
         double angle = Math.atan2(y1, x1);

         robot.frontLeft.setPower(Math.sin(-angle + Math.PI/4) - x2);
         robot.frontRight.setPower(Math.cos(-angle + Math.PI/4) + x2);
         robot.backLeft.setPower(Math.cos(-angle + Math.PI/4) - x2);
         robot.backRight.setPower(Math.sin(-angle + Math.PI/4) + x2);

         **/


    }
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

    private static double clamp(double val, double min, double max) {

        return Math.max(min, Math.min(max, val));
    }
}

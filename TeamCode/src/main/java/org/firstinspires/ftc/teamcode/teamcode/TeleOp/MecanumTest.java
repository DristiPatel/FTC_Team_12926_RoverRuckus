package org.firstinspires.ftc.teamcode.teamcode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.HardwareRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.lang.Math;

public class MecanumTest extends OpMode {


    private enum DriveSpeed{

        VERY_SLOW, SLOW, FAST, VERY_FAST

    }

    private enum LiftControl{



    }

    private enum ArmExtensionControl{


    }

    private enum ArmRotationControl{

    }






    HardwareRobot robot;


    @Override
    public void init(){

        robot = new HardwareRobot(hardwareMap);
    }

    @Override
    public void loop(){

        Telemetry();

        DriveControl();

    }

    private void Telemetry(){

    //display run data here

    }

    //sets power to motors from joystick input based on mecanum setup
    public void DriveControl(){

        //Reverse the y coordinate
        double x1 = gamepad1.left_stick_x;
        double y1 = -gamepad1.left_stick_y;
        float x2 = gamepad1.right_stick_x;

        //Left joystick controls translation, right joystick controls turning
        robot.frontLeft.setPower(clamp(y1 + x1 + x2, -1, 1));
        robot.frontRight.setPower(clamp(y1 - x1 - x2, -1, 1));
        robot.backLeft.setPower(clamp(y1 - x1 + x2, -1, 1));
        robot.backRight.setPower(clamp(y1 + x1 -x2, -1, 1));


        // trig implementation for mecanum drive
        /**
        double angle = Math.atan2(y1, x1);

        robot.frontLeft.setPower(Math.sin(-angle + Math.PI/4) - x2);
        robot.frontRight.setPower(Math.cos(-angle + Math.PI/4) + x2);
        robot.backLeft.setPower(Math.cos(-angle + Math.PI/4) - x2);
        robot.backRight.setPower(Math.sin(-angle + Math.PI/4) + x2);

        **/


    }


    //Check d-pad for speed modifier
    public void CheckSpeed(){


    }

    //clamps values outside of the range to the min/max
    private static double clamp(double val, double min, double max) {

        return Math.max(min, Math.min(max, val));
    }
}

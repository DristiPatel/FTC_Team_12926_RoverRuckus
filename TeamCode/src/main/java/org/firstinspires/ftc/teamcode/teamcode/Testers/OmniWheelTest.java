package org.firstinspires.ftc.teamcode.teamcode.Testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name ="omni wheels are gay", group="Linear Opmode" )

public class OmniWheelTest extends LinearOpMode {

   private DcMotor left, right, middle;

   @Override
    public void runOpMode(){

       left = hardwareMap.get(DcMotor.class, "Left Motor");
       right = hardwareMap.get(DcMotor.class, "Right Motor");
       middle = hardwareMap.get(DcMotor.class, "Middle Motor");


       right.setDirection(DcMotor.Direction.REVERSE);

       waitForStart();

       while(opModeIsActive()){


           left.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
           right.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);

           middle.setPower(gamepad1.left_stick_x);

       }

   }

}

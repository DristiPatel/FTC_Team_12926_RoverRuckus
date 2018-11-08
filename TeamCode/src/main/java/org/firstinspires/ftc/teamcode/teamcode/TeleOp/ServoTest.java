package org.firstinspires.ftc.teamcode.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Servo Test", group="Linear Opmode")
public class ServoTest extends OpMode {


    private Servo servo1, servo2;

    public void init(){

        servo1 = hardwareMap.get(Servo.class, "Servo1");
        servo2 = hardwareMap.get(Servo.class, "Servo2");
    }


    public void loop(){

        if(gamepad1.dpad_up){

            servo1.setPosition(0);
            servo2.setPosition(.5);

        }else if(gamepad1.dpad_down){

            servo1.setPosition(.5);
            servo2.setPosition(1);

        }


    }


}

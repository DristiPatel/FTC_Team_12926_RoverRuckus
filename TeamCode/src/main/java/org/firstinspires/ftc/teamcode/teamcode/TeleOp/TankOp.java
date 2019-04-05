package org.firstinspires.ftc.teamcode.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name="Testeroni", group="TeleOp")
public class TankOp extends OpMode {


    private DcMotor latchMotor, slideMotor, intakeMotor, liftMotor,
            leftBack, rightBack, leftFront, rightFront;

    private Servo lflipServo, rflipServo, blockServo, boxServo;

    private int maxSlide = 10000,
            minSlide = 0,
            maxLift = 10000,
            minLift = 0;

    private int speedMod = 1;

    boolean usingLimits = true;

    //DogeVuforia doge;


    public void init(){

        latchMotor = hardwareMap.dcMotor.get("latch");
        slideMotor = hardwareMap.dcMotor.get("slide");
        intakeMotor = hardwareMap.dcMotor.get("intake");
        liftMotor = hardwareMap.dcMotor.get("lift");

        leftBack = hardwareMap.dcMotor.get("lb");
        rightBack = hardwareMap.dcMotor.get("rb");
        leftFront = hardwareMap.dcMotor.get("lf");
        rightFront = hardwareMap.dcMotor.get("rf");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        lflipServo = hardwareMap.servo.get("flipl");
        rflipServo = hardwareMap.servo.get("flipr");
        blockServo = hardwareMap.servo.get("block");
        boxServo = hardwareMap.servo.get("box");

        if(gamepad1.start && (gamepad2.start || gamepad1.a))
            usingLimits = false;

        //doge = new DogeVuforia(hardwareMap);

    }

    public void loop(){

        telemetry.addData("latch", latchMotor.getCurrentPosition());
        telemetry.addData("slide", slideMotor.getCurrentPosition());
        telemetry.addData("intake", intakeMotor.getCurrentPosition());

        telemetry.addLine("Gold Detector stuffs0-----------");
        //telemetry.addData("is Aligned: ", doge.getIsAligned());
        //telemetry.addData("X pos: ", doge.getGoldXPosition());
        //telemetry.addData("Y pos: ", doge.getPosition());


        // Update telemetry
        telemetry.update();

        DriveControl();

        LatchControl();

        FlipControl();

        SlideControl();

        IntakeControl();

        LiftControl();

        BlockControl();

    }

    public void DriveControl(){

        leftFront.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.9), -1, 1) * speedMod);
        rightFront.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.9), -1, 1) * speedMod);
        leftBack.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.9), -1, 1) * speedMod);
        rightBack.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.9), -1, 1) * speedMod);

    }

    public void LiftControl(){

        if((liftMotor.getCurrentPosition() > minLift || !usingLimits) && gamepad2.left_bumper)
            liftMotor.setPower(-1);
        else if((liftMotor.getCurrentPosition() < maxLift || !usingLimits) && gamepad2.right_bumper)
            liftMotor.setPower(1);
        else
            liftMotor.setPower(0);

    }

    public void LatchControl(){

        if (gamepad1.dpad_up){
            latchMotor.setPower(1);
        }else if (gamepad1.dpad_down){
            latchMotor.setPower(-1);
        }else{
            latchMotor.setPower(0);
        }
    }

    public void SlideControl(){

        if((slideMotor.getCurrentPosition() > minSlide || !usingLimits) && gamepad2.left_stick_y < 0)
            slideMotor.setPower(gamepad2.left_stick_y);
        else if((slideMotor.getCurrentPosition() < maxSlide || !usingLimits) && gamepad2.left_stick_y > 0)
            slideMotor.setPower(gamepad2.left_stick_y);
        else
            slideMotor.setPower(0);

    }

    public void FlipControl(){

        if(gamepad2.x){

            lflipServo.setPosition(1);
            rflipServo.setPosition(-1);

        }else if(gamepad2.y){

            lflipServo.setPosition(-1);
            rflipServo.setPosition(1);
        }

    }

    public void BlockControl(){

        if(gamepad2.a){

            blockServo.setPosition(-1);
            boxServo.setPosition(-1);

        }else if(gamepad2.b){

            blockServo.setPosition(1);
            boxServo.setPosition(1);
        }

    }

    public void IntakeControl(){

        if(gamepad2.left_trigger != 0)
            intakeMotor.setPower(1);
        else if(gamepad2.right_trigger != 0)
            intakeMotor.setPower(-1);
        else
            intakeMotor.setPower(0);

    }

}




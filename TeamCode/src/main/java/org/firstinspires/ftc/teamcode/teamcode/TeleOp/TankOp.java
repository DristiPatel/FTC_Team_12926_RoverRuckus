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

    private Servo blockServo, boxServo;
    private CRServo lflipServo, rflipServo;

    private int maxSlide = 2000,
            minSlide = 0,
            maxLift = 4400,
            minLift = 0;

    private int speedMod = 1;

    boolean usingLimits = true, locked = false, changeLock = false;

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

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        latchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        lflipServo = hardwareMap.crservo.get("flipl");
        rflipServo = hardwareMap.crservo.get("flipr");
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
        telemetry.addData("lift", liftMotor.getCurrentPosition());

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

        leftFront.setPower(Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.9), -1, 1) * speedMod);
        rightFront.setPower(Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.9), -1, 1) * speedMod);
        leftBack.setPower(Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.9), -1, 1) * speedMod);
        rightBack.setPower(Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x * 0.9), -1, 1) * speedMod);

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

        if((slideMotor.getCurrentPosition() > minSlide || !usingLimits) && gamepad2.right_stick_x < 0)
            slideMotor.setPower(gamepad2.right_stick_x * 0.5);
        else if((slideMotor.getCurrentPosition() < maxSlide || !usingLimits) && gamepad2.right_stick_x > 0)
            slideMotor.setPower(gamepad2.right_stick_x * 0.5);
        else
            slideMotor.setPower(0);

    }

    public void FlipControl(){

        if(gamepad2.left_stick_y > 0){

            lflipServo.setPower(1);
            rflipServo.setPower(-1);

        }else if(gamepad2.left_stick_y < 0){

            lflipServo.setPower(-1);
            rflipServo.setPower(1);

        }else{

            lflipServo.setPower(0);
            rflipServo.setPower(0);

        }

    }

    public void BlockControl(){

        if(gamepad2.a && !changeLock){

            changeLock = true;

        }

        if(changeLock && !gamepad2.a){

            changeLock = false;
            locked = !locked;

        }

        if(locked) {

            blockServo.setPosition(-1);
            boxServo.setPosition(-1);

        }else {

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




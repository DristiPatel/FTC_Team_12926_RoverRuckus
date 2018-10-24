package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class HardwareRobot {

    public DcMotor frontLeft, frontRight, backLeft, backRight;

    public DcMotor liftMotor, extensionMotor, rotationMotor, collectionMotor;

    public Servo latch;

    public ColorSensor colorSensor;

    //Initialize and map the motors to the robot
    public HardwareRobot(HardwareMap hwmp){

        frontLeft = hwmp.dcMotor.get("Front Left");
        frontRight = hwmp.dcMotor.get("Front Right");
        backLeft = hwmp.dcMotor.get("Back Left");
        backRight = hwmp.dcMotor.get("Back Right");





    }

    public void ResetAllEncoders(){


        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);





    }

    public void ResetDriveEncoders(){

         StopDriveMotors();

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        collectionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void StopAllMotors(){

        liftMotor.setPower(0);
        extensionMotor.setPower(0);
        rotationMotor.setPower(0);
        collectionMotor.setPower(0);

    }

    public void StopDriveMotors(){

        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);

    }


}

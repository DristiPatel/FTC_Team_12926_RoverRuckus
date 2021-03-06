/**
 * @author Dristi Patel
 * @author Hanaa Siddiqui
 *
 * @version 1.0.0
 * @date 11/6/2018
 */

//Importing different motors
package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A HardwareRobot has eight dc motors, one  servo, and one color sensor.  It can map the hardware, reset encoders, and stop motors
 */
public class HardwareRobot {


    //Instance Fields
    public DcMotor leftDrive, rightDrive, strafeDrive;

    public DcMotor liftMotor, extensionMotor, collectionMotor;//, rotationMotor; //motors for the other functions of the robot

    public Servo markerServo, swingServo1, swingServo2, scoopServo, webcamServo; //servo for the latch feature of the robot



    //Constructors
    /**
     * Initializes and maps the motors to the robots
     * @param hwmp
     */
    public HardwareRobot(HardwareMap hwmp){


        //DRIVE MOTORS
        leftDrive = hwmp.dcMotor.get("Left Drive");
        rightDrive = hwmp.dcMotor.get("Right Drive");
        strafeDrive = hwmp.dcMotor.get("Strafe Drive");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


        //OTHER MOTORS
        liftMotor = hwmp.dcMotor.get("Lift Motor");

        extensionMotor = hwmp.dcMotor.get("Extension Motor");

        collectionMotor = hwmp.dcMotor.get("Collection Motor");


        //SERVOS
        swingServo1 = hwmp.servo.get("Swing Servo1");
        swingServo2 = hwmp.servo.get("Swing Servo2");

        scoopServo = hwmp.servo.get("Scoop Servo");

        webcamServo = hwmp.servo.get("Webcam Servo");

        markerServo = hwmp.servo.get("Marker Servo");



        //reset encoders
        //ResetAllEncoders();

    }
    //Mutator methods
    /**
     * Stops all non-wheel motors before setting them to run with encoders
     */
    public void ResetAllEncoders(){

        ResetDriveEncoders();

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        collectionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * Stops all motors before setting the non-wheel motors to run with encoders
     */
    public void ResetDriveEncoders(){

        StopDriveMotors();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        strafeDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * Sets the power of all motors to 0
     */
    public void StopAllMotors(){

        StopDriveMotors();

        liftMotor.setPower(0);
        extensionMotor.setPower(0);

        collectionMotor.setPower(0);

    }

    /**
     * Sets power of wheel motors to 0
     */
    public void StopDriveMotors(){

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        strafeDrive.setPower(0);


    }

    public void ResetServos(){


        scoopServo.setPosition(Servo.MIN_POSITION);

    }


}

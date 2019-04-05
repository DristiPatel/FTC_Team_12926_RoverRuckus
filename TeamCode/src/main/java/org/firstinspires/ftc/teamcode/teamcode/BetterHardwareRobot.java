/**
 * @author Dristi Patel
 * @author Hanaa Siddiqui
 *
 * @version 1.0.0
 * @date 11/6/2018
 */

//Importing different motors
package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A HardwareRobot has eight dc motors, one  servo, and one color sensor.  It can map the hardware, reset encoders, and stop motors
 */
public class BetterHardwareRobot {


    //Instance Fields
    public DcMotor frontLeft, frontRight, backLeft, backRight;

    public DcMotor liftMotor, extensionMotor, scoopMotor, flipMotor; //motors for the other functions of the robot

    public CRServo intakeServo;//continuous servo for intake

    public Servo scoopServo, markerServo, webcamServo;// flips container over

    //Constructors
    /**
     * Initializes and maps the motors to the robots
     * @param hwmp
     */
    public BetterHardwareRobot(HardwareMap hwmp){


        //DRIVE MOTORS
        frontLeft = hwmp.dcMotor.get("Front Left");
        frontRight = hwmp.dcMotor.get("Front Right");
        backLeft = hwmp.dcMotor.get("Back Left");
        backRight = hwmp.dcMotor.get("Back Right");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);


        //OTHER MOTORS

        liftMotor = hwmp.dcMotor.get("Lift Motor");
        extensionMotor = hwmp.dcMotor.get("Extension Motor");
        scoopMotor = hwmp.dcMotor.get("Scoop Motor");
        flipMotor = hwmp.dcMotor.get("Flip Motor");


        //SERVOS
        intakeServo = hwmp.crservo.get("Intake Servo");
        scoopServo = hwmp.servo.get("Scoop Servo");
        markerServo = hwmp.servo.get("Marker Servo");
        webcamServo = hwmp.servo.get("Webcam Servo");



    }
    //Mutator methods
    /**
     * Stops all non-wheel motors before setting them to run with encoders
     */
    public void ResetAllEncoders(){

        ResetDriveEncoders();

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        scoopMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flipMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scoopMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Stops all motors before setting the non-wheel motors to run with encoders
     */
    public void ResetDriveEncoders(){

        StopDriveMotors();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    /**
     * Sets the power of all motors to 0
     */
    public void StopAllMotors(){

        StopDriveMotors();

        liftMotor.setPower(0);
        extensionMotor.setPower(0);


    }

    /**
     * Sets power of wheel motors to 0
     */
    public void StopDriveMotors(){

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }


    public void ResetServos(){


    }


}

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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A HardwareRobot has eight dc motors, one  servo, and one color sensor.  It can map the hardware, reset encoders, and stop motors
 */
public class HardwareRobot {


    //Instance Fields
    public DcMotor frontLeft, frontRight, backLeft, backRight; //motors for the wheels

    public DcMotor liftMotor, extensionMotor, rotationMotor, collectionMotor; //motors for the other functions of the robot

    //public Servo markerServo; //servo for the latch feature of the robot



    //Constructors
    /**
     * Initializes and maps the motors to the robots
     * @param hwmp
     */
    public HardwareRobot(HardwareMap hwmp){

        frontLeft = hwmp.dcMotor.get("Front Left");
        frontRight = hwmp.dcMotor.get("Front Right");
        backLeft = hwmp.dcMotor.get("Back Left");
        backRight = hwmp.dcMotor.get("Back Right");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        liftMotor = hwmp.dcMotor.get("Lift Motor");
        extensionMotor = hwmp.dcMotor.get("Extension Motor");
        rotationMotor = hwmp.dcMotor.get("Rotation Motor");
        collectionMotor = hwmp.dcMotor.get("Collection Motor");
        //markerServo = hwmp.servo.get("Marker Servo");

        ResetAllEncoders();

    }
    //Mutator methods
    /**
     * Stops all non-wheel motors before setting them to run with encoders
     */
    public void ResetAllEncoders(){


        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rotationMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    /**
     * Stops all motors before setting the non-wheel motors to run with encoders
     */
    public void ResetDriveEncoders(){

        StopDriveMotors();

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        rotationMotor.setPower(0);
        collectionMotor.setPower(0);

    }

    /**
     * Sets power of wheel motors to 0
     */
    public void StopDriveMotors(){

        backLeft.setPower(0);
        backRight.setPower(0);
        frontRight.setPower(0);
        frontLeft.setPower(0);

    }


}

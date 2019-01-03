package org.firstinspires.ftc.teamcode.teamcode.Testers;

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.Autonomous.AutonomousRobot;
import org.firstinspires.ftc.teamcode.teamcode.HardwareRobot;

@TeleOp(name = "Doge scoot follow gold", group = "Doge")

@Disabled
public class DogeTest extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private GoldAlignDetector detector;


    @Override
    public void runOpMode(){

        frontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        frontRight = hardwareMap.get(DcMotor.class, "Front Right");
        backLeft = hardwareMap.get(DcMotor.class, "Back Left");
        backRight = hardwareMap.get(DcMotor.class, "Back Right");

        //reverse a side of motors
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);




        //Initialize and set the settings for the detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!


        //Wait for start and RUN
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Is Aligned?: ", detector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("X Pos: ", detector.getXPosition()); // Gold X position.

            telemetry.update();


            if(detector.isFound())

                {
                if (!detector.getAligned() && detector.getXPosition() < 270) {

                    //move right
                    frontLeft.setPower(.3);
                    frontRight.setPower(-.3);
                    backLeft.setPower(-.3);
                    backRight.setPower(.3);


                } else if (!detector.getAligned() && detector.getXPosition() > 270) {

                    //move left
                    frontLeft.setPower(-.3);
                    frontRight.setPower(.3);
                    backLeft.setPower(.3);
                    backRight.setPower(-.3);

                } else {

                    //stop robot if aligned
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);

                }
            }else{
                //stop robot if no gold cube in sight
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);


            }


        }

        detector.disable();
    }





}

package org.firstinspires.ftc.teamcode.teamcode.Testers;


import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.teamcode.Autonomous.AutonomousRobot;

@Disabled
@TeleOp(name = "doge test", group = "TeleOp")


public class DetectorTest extends LinearOpMode {

    WebcamName webcamName;

    private GoldAlignDetector detector;

    @Override
    public void runOpMode(){

        webcamName = hardwareMap.get(WebcamName.class, "Webcam");


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


        waitForStart();


        while (opModeIsActive()) {

            telemetry.addData("Is Aligned?: ", detector.getAligned()); // Is the bot aligned with the gold mineral?
            telemetry.addData("X Pos: ", detector.getXPosition()); // Gold X position.
            telemetry.addData("Y Pos: ", detector.getScreenPosition().y);// Gold Y pos

            telemetry.update();
        }


    }



}

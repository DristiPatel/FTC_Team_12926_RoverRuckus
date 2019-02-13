package org.firstinspires.ftc.teamcode.teamcode.Autonomous;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "Depot Side Autonomous", group = "Autonomous")
public class AutoBotDepot extends BetterAutonomousRobot{


    ElapsedTime runTime = new ElapsedTime();
    ElapsedTime waitTime = new ElapsedTime();

    double retractTime;


    @Override
    public void runOpMode(){

        super.runOpMode();



        //IMU STUFF
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);


        while (!imu.isGyroCalibrated()){

            telemetry.addLine("Calibrating...");
            telemetry.addData("heading: ", imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.update();
        }

        telemetry.addLine("Gyro ready");
        telemetry.addData("heading: ",imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();


        waitForStart();



        TimeDrive(180, .5, 1);

        TimeDrive(90, .25, .5);

        TimeDrive(0, .6, 1);

        Sampling();

        AbsoluteTurn(.3, baseAngle);

        TimeDrive(90, .5, .5);

        dogevuforia.stop();

    }


}

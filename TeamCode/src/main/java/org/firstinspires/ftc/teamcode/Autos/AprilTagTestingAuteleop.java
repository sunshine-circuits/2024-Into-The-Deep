package org.firstinspires.ftc.teamcode.Autos;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.InchAutoParent;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

@Autonomous

public class AprilTagTestingAuteleop extends InchAutoParent {

    @Override
    public void InchDrive(double distance, double angle, double pow, String telem){
        double pulses = InchesToPulses(distance);
        double diameterOfWheels = 104.0 / 25.4;
        double pulsesInX = ((Math.cos((angle*Math.PI)/180.0) * distance) / (Math.PI * diameterOfWheels)) * 537.7;
        double pulsesInY = ((Math.sin((angle*Math.PI)/180.0) * distance) / (Math.PI * diameterOfWheels)) * 537.7;
        driverInteruptable((int)(pulsesInY - pulsesInX), (int)(pulsesInY + pulsesInX), (int)(pulsesInY + pulsesInX), (int)(pulsesInY - pulsesInX), pow, telem);
        //FR, FL, BR, BL
    }
    public void InchRuptTags(double distance, double angle, double pow, String telem, AprilTagProcessor tagProcessor){
        InchDrive(distance, angle, pow, telem);
        boolean check = false;
        while((FRMotor.isBusy()||FLMotor.isBusy()||BRMotor.isBusy()||BLMotor.isBusy())&&!check) {
            if (tagProcessor.getDetections().size() > 0) {
                for (int i = 0; i < tagProcessor.getDetections().size(); i++) {
                    AprilTagDetection tag = tagProcessor.getDetections().get(i);
                    if ((tag.id == 16)||(tag.id == 13)) {
                        check = true;
                    }
                }
            }
        }
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        powerlevel=0.9;
        telemetry.addData("Motors","Grabbing");
        telemetry.update();
        FRMotor = hardwareMap.get(DcMotor.class,"FRMotor");
        FLMotor = hardwareMap.get(DcMotor.class,"FLMotor");
        BRMotor = hardwareMap.get(DcMotor.class,"BRMotor");
        BLMotor = hardwareMap.get(DcMotor.class,"BLMotor");
        telemetry.addData("Motors","Got");
        telemetry.update();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagID(true).setDrawTagOutline(true).setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11).setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()).build();
        VisionPortal visionPortal= new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam")).setCameraResolution(new Size(640,480)).build();

        myIMU = hardwareMap.get(IMU.class,"imu");

        waitForStart();
        IMU.Parameters myIMUParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        myIMU.initialize(myIMUParameters);
        visionPortal.resumeLiveView();
        visionPortal.resumeStreaming();
        if (opModeIsActive()) {
            myIMU.resetYaw();
            telemetry.addData("State", visionPortal.getCameraState());
            InchRuptTags(100, 0, 1, "Big Go.", tagProcessor);
        }
    }





}
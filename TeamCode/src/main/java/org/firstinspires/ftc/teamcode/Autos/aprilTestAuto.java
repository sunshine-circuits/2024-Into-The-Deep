package org.firstinspires.ftc.teamcode.Autos;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.Driver;
import org.firstinspires.ftc.teamcode.utils.Keybind;
import org.firstinspires.ftc.teamcode.utils.RobotConfig;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.apriltag.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name= "AprilTagTestTeleAuto")
public class aprilTestAuto extends LinearOpMode {

    public DcMotor frontRightWheel;
    public DcMotor frontLeftWheel;
    public DcMotor backRightWheel;
    public DcMotor backLeftWheel;
    public double speedMultiplier =1;
    public double fx = 1500;
    public double fy = 1500;
    public double cx = 970;
    public double cy = 390;
    public final boolean calibratedCamera=false;
    private AprilTagProcessor aprilTagProcessor;

    @Override
    public void runOpMode() throws InterruptedException {
    //640x480
        //if(calibratedCamera){
        //    aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        //}else{
        aprilTagProcessor = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagID(true).setDrawTagOutline(true).setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11).setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()).build();
        //}
        VisionPortal visionPortal = new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")).addProcessor(aprilTagProcessor).setCameraResolution(new Size(640,480)).build();

        frontRightWheel = hardwareMap.get(DcMotor.class,"FRMotor");
        frontLeftWheel = hardwareMap.get(DcMotor.class,"FLMotor");
        backRightWheel = hardwareMap.get(DcMotor.class,"BRMotor");
        backLeftWheel = hardwareMap.get(DcMotor.class,"BLMotor");
        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double frontRightTargetPow=0;
        double frontLeftTargetPow=0;
        double backRightTargetPow=0;
        double backLeftTargetPow=0;
        waitForStart();

        while(opModeIsActive()){
            //Motors

            //movement for forward&back,      left&right            Rotation
            frontRightTargetPow = 0 + gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            frontRightWheel.setPower(speedMultiplier*frontRightTargetPow);
            telemetry.addData("Front Right TP", frontRightTargetPow);
            telemetry.addData("Front Right CP", frontRightWheel.getPower());

            frontLeftTargetPow = 0 - gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            frontLeftWheel.setPower(speedMultiplier*frontLeftTargetPow);
            telemetry.addData("Front Left TP", frontLeftTargetPow);
            telemetry.addData("Front Left CP", frontLeftWheel.getPower());

            backRightTargetPow = 0 + gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
            backRightWheel.setPower(speedMultiplier*backRightTargetPow);
            telemetry.addData("Back Right TP", backRightTargetPow);
            telemetry.addData("Back Right CP", backRightWheel.getPower());

            backLeftTargetPow = 0 - gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
            backLeftWheel.setPower(speedMultiplier*backLeftTargetPow);
            telemetry.addData("Back Left TP", backLeftTargetPow);
            telemetry.addData("Back Left CP", backLeftWheel.getPower());

            if (gamepad1.right_trigger>=0.5){
                if (speedMultiplier <1){
                    speedMultiplier = speedMultiplier +0.1;
                }
                while (gamepad1.right_trigger>=0.5){

                }
            }
            else if (gamepad1.left_trigger>=0.5){
                if (speedMultiplier >0.1){
                    speedMultiplier = speedMultiplier -0.1;
                }
                while (gamepad1.left_trigger>=0.5){

                }
            }
            telemetry.addData("Speed Multiplier:", speedMultiplier);


            for(AprilTagDetection detection :aprilTagProcessor.getDetections()){
                if(detection.metadata !=null){
                    telemetry.addLine(String.format("\n==== (ID %d) %s",detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                }else{
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown",detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x,detection.center.y));
                }
            }

            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), X (Up) dist.");
            telemetry.addLine("PRY = Pith, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");
            telemetry.update();
            sleep(20);
        }
    }
}

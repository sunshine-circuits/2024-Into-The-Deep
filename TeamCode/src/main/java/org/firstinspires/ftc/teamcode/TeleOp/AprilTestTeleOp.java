package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.utils.InchAutoParent;
import org.firstinspires.ftc.teamcode.utils.Coordinate;

@TeleOp
public class AprilTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagID(true).setDrawTagOutline(true).setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11).setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()).build();
        VisionPortal visionPortal= new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam")).setCameraResolution(new Size(640,480)).build();
<<<<<<< HEAD

=======
        IMU myIMU;
        myIMU = hardwareMap.get(IMU.class,"imu");
>>>>>>> 441cd1f7d83c49733297ab80bebf06019eb23eed
        waitForStart();
        while (!isStopRequested()&&opModeIsActive()){
            if(tagProcessor.getDetections().size()>0){
                for(int i=0; i<tagProcessor.getDetections().size(); i++) {
                    AprilTagDetection tag = tagProcessor.getDetections().get(i);
                    try {
                        telemetry.addData(i + " x", tag.ftcPose.x);
                        telemetry.addData(i + " y", tag.ftcPose.y);
                        telemetry.addData(i + " z", tag.ftcPose.z);
                        telemetry.addData(i + " roll", tag.ftcPose.roll);
                        telemetry.addData(i + " pitch", tag.ftcPose.pitch);
                        telemetry.addData(i + " yaw", tag.ftcPose.yaw);
<<<<<<< HEAD
//                        telemetry.addData();
=======

>>>>>>> 441cd1f7d83c49733297ab80bebf06019eb23eed
                    } catch (Exception e) {
                        telemetry.addData(i + " x", "NULL");
                        telemetry.addData(i + " y", "NULL");
                        telemetry.addData(i + " z", "NULL");
                        telemetry.addData(i + " roll", "NULL");
                        telemetry.addData(i + " pitch", "NULL");
                        telemetry.addData(i + " yaw", "NULL");
                    }
                    telemetry.addData("IMU yaw", myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                }
            }
            telemetry.update();
        }
    }
}

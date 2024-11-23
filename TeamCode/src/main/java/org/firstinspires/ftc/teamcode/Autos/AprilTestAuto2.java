package org.firstinspires.ftc.teamcode.Autos;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name= "AprilTagTest")
public class AprilTestAuto2 extends LinearOpMode {

    public double fx = 1500;
    public double fy = 1500;
    public double cx = 970;
    public double cy = 390;
    public final boolean calibratedCamera=false;
    private AprilTagProcessor aprilTagProcessor;
    @Override
    public void runOpMode() throws InterruptedException {
        //640x480
        if(calibratedCamera){
            aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        }else{
            aprilTagProcessor = new AprilTagProcessor.Builder().setLensIntrinsics(fx,fy,cx,cy).build();
        }
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640,480))
                .build();


        waitForStart();

        while(opModeIsActive()){
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

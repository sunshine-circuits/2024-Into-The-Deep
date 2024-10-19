package org.firstinspires.ftc.teamcode.Autos;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="FirstAuto")

public class FirstAuto extends LinearOpMode{
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor ArmJointMotor;



    //    @Override



    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("Motors","Grabbing");
        telemetry.update();
        FRMotor = hardwareMap.get(DcMotor.class,"FRMotor");
        FLMotor = hardwareMap.get(DcMotor.class,"FLMotor");
        BRMotor = hardwareMap.get(DcMotor.class,"BRMotor");
        BLMotor = hardwareMap.get(DcMotor.class,"BLMotor");
        ArmJointMotor = hardwareMap.get(DcMotor.class,"ArmJointMotor");
        telemetry.addData("Motors","Got");
        telemetry.update();


        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagID(true).setDrawTagOutline(true).setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11).setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()).build();
        VisionPortal visionPortal= new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam")).setCameraResolution(new Size(640,480)).build();
        waitForStart();
        if (opModeIsActive()) {
            /*
            driver(-1100,1100,1100,-1100,0.1, "Right");
            driver(3550,3550,3550,3550,0.1, "Big Forward");
            driverInteruptable(500,-500,-500,500,0.05, "interuptable left");
            outerloop:
            while(FRMotor.isBusy()||FLMotor.isBusy()||BRMotor.isBusy()||BLMotor.isBusy()) {
                if (tagProcessor.getDetections().size() > 0) {
                    for (int i = 0; i < tagProcessor.getDetections().size(); i++) {
                        AprilTagDetection tag = tagProcessor.getDetections().get(i);
                        if (tag.id == 13) {
                            break outerloop;
                        }
                    }
                }
            }
            driver(0,0,0,0,0,"Stopping");
            driver(-500,-500,-500,-500,0.1,"Back");
            driver(-1800,1800,1800,-1800,0.1, "BIG RIGHT");
            driver(-1000,-1000,-1000,-1000,0.1,"Back");
            SingleMotorDriver(ArmJointMotor, 300,1,"Arm in position");
            */
            InchDrive(24,0,1,"RUN RABBIT RUN RABIT RUN RUN RUN");

            telemetry.addData("Moves","Done");
            telemetry.update();
            // RESET

        }
    }
}
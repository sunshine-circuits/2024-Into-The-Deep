package org.firstinspires.ftc.teamcode.Autos;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.InchAutoParent;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="BucketZoneAutoBlue")

public class BucketAutoRed extends InchAutoParent {

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("Motors","Grabbing");
        telemetry.update();
        FRMotor = hardwareMap.get(DcMotor.class,"FRMotor");
        FLMotor = hardwareMap.get(DcMotor.class,"FLMotor");
        BRMotor = hardwareMap.get(DcMotor.class,"BRMotor");
        BLMotor = hardwareMap.get(DcMotor.class,"BLMotor");
        ArmJointMotor = hardwareMap.get(DcMotor.class,"ArmJointMotor");
        Headlight = hardwareMap.get(Servo.class, "Headlight");
        telemetry.addData("Motors","Got");
        telemetry.update();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagID(true).setDrawTagOutline(true).setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11).setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()).build();
        VisionPortal visionPortal= new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam")).setCameraResolution(new Size(640,480)).build();
        Headlight.setPosition(0);
        waitForStart();
        /*
        while(ArmHomeSensor.isPressed()==false){
            ArmJointMotor.setPower(-0.25);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        ArmJointMotor.setPower(0);
        */
        Headlight.setPosition(1);
        if (opModeIsActive()) {
            InchDrive(2,0,powerlevel, "Right");
            InchDrive(3.5,90,powerlevel, "Big Forward");

            driverInteruptable(400,-400,-400,400,powerlevel/2, "interuptable left");
            Headlight.setPosition(1);
            outerloop:
            while(FRMotor.isBusy()||FLMotor.isBusy()||BRMotor.isBusy()||BLMotor.isBusy()) {
                if (tagProcessor.getDetections().size() > 0) {
                    for (int i = 0; i < tagProcessor.getDetections().size(); i++) {
                        AprilTagDetection tag = tagProcessor.getDetections().get(i);
                        if (tag.id == 16) {
                            break outerloop;
                        }
                    }
                }
            }
            Headlight.setPosition(0);
            driver(0,0,0,0,powerlevel/2,"Stopping");
            //*/
            InchDrive(58,0,powerlevel,"RightOnceMore");
            driver(1975,-1975,1975,-1975, powerlevel, "rotation");
            InchDrive(12,90,powerlevel,"Back");
            SingleMotorDriver(ArmJointMotor, 500,1,"Arm almost ready to fire");
            SingleMotorDriver(ArmJointMotor, 500,-1,"Arm almost ready to fire");


            telemetry.addData("Moves","Done");
            telemetry.update();
            // RESET

        }
    }
}
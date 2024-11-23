package org.firstinspires.ftc.teamcode.Autos;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.InchAutoParent;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="BucketZoneAuto")

public class BucketAuto extends InchAutoParent {

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

        AprilTagProcessor tagProcessor = new AprilTagProcessor
                .Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();
        VisionPortal visionPortal= new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(640,480))
                .build();
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
            InchDrive(22.5,90,powerlevel, "Big Forward");
            /*
            driverInteruptable(400,-400,-400,400,powerlevel/2, "interuptable left");
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
            driver(0,0,0,0,powerlevel/2,"Stopping");
            */
            InchDrive(19,-90,powerlevel,"Back");
            InchDrive(57,0,powerlevel,"RightOnceMore");
            driver(1975,-1975,1975,-1975, powerlevel, "rotation");
            InchDrive(12,90,powerlevel,"Back");
            SingleMotorDriver(ArmJointMotor, 500,1,"Arm almost ready to fire");
            SingleMotorDriver(ArmJointMotor, 500,-1,"Arm almost ready to fire");


            telemetry.addData("Moves","Done");
            telemetry.update();
            // RESET

        }
    }
    private void drivepathred(double rx, double ry, double rot, double pow, String telem)
    {
        double robotwidth = 18;
        /*The rx is "Robot x" in relation to the field(blue observation zone is (0, 0))
          The ry is "Robot y" in relation to the field(blue observation zone is (0, 0))
          The rot is the "Robot rotation" in relation to the feild(0 degrees is straight 'upward')
          The robot always starts the autonomous touching the edge!
        */
        DriveIrrespectiveOfAngle(rot, 108-rx, 36-ry, pow, telem);
        //set it to be (108, 36)
        RotateDegrees(0-rot, pow, telem);
        //Make Bot straight
        DistanceDrive(-24, 0, pow, telem);
        //set bot to be (84, 36)
        DistanceDrive(0, -12, pow, telem);
        //set bot to be (84, 24)
        DistanceDrive(36-(robotwidth/2), 0, pow, telem);
        //set bot to be (120-(robotwidth/2), 24)
        RotateDegrees(45, pow, telem);
        //Rotate the robot 45 degrees clockwise
        DistanceDrive(6+(robotwidth/2), -6, pow, telem);
        //set bot to be (126, 18)
    }
    private void drivepathblue(double rx, double ry, double rot, double pow, String telem)
    {
        double robotwidth = 18;
        /*The rx is "Robot x" in relation to the field(blue observation zone is (0, 0))
          The ry is "Robot y" in relation to the field(blue observation zone is (0, 0))
          The rot is the "Robot rotation" in relation to the feild(0 degrees is straight 'upward')
          The robot always starts the autonomous touching the edge!
        */
        DriveIrrespectiveOfAngle(rot, 36-rx, 108-ry, pow, telem);
        //set it to be (36, 108)
        RotateDegrees(0-rot, pow, telem);
        //Make Bot straight
        DistanceDrive(24, 0, pow, telem);
        //set bot to be (60, 108)
        DistanceDrive(0, 12, pow, telem);
        //set bot to be (60, 120)
        DistanceDrive(-36+(robotwidth/2), 0, pow, telem);
        //set bot to be (24+(robotwidth/2), 120)
        RotateDegrees(-45, pow, telem);
        //Rotate the robot 45 degrees counterclockwise
        DistanceDrive(-6-(robotwidth/2), 6, pow, telem);
        //set bot to be (18, 126)
    }
}
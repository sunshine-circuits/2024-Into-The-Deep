package org.firstinspires.ftc.teamcode.Autos;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.InchAutoParent;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="BucketZoneAutoBlue")
public class BucketAutoBlue extends InchAutoParent {

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("Motors","Grabbing");
        telemetry.update();
        FRMotor = hardwareMap.get(DcMotor.class,"FRMotor");
        FLMotor = hardwareMap.get(DcMotor.class,"FLMotor");
        BRMotor = hardwareMap.get(DcMotor.class,"BRMotor");
        BLMotor = hardwareMap.get(DcMotor.class,"BLMotor");
        RightServo = hardwareMap.get(Servo.class, "RightServo");
        LeftServo = hardwareMap.get(Servo.class, "LeftServo");
        ArmJointMotor = hardwareMap.get(DcMotor.class,"ArmJointMotor");
        ArmExtendMotor = hardwareMap.get(DcMotor.class,"ArmExtendMotor");
        Headlight = hardwareMap.get(Servo.class, "Headlight");
        telemetry.addData("Motors","Got");
        telemetry.update();

        ArmExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmJointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmJointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
        if (opModeIsActive()) {
            CloseClaws();
            InchDrive(2,0,powerlevel, "Right");
            ArmJointMotor.setTargetPosition(750);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(1);
            ArmExtendMotor.setTargetPosition(-3000);
            ArmExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmExtendMotor.setDirection(DcMotor.Direction.FORWARD);
            ArmExtendMotor.setPower(1);
            while((ArmExtendMotor.getCurrentPosition()>-2975)||(ArmJointMotor.getCurrentPosition()<700)){
                telemetry.addData("ExtendPosition: ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("JointPosition: ",ArmJointMotor.getCurrentPosition());
                telemetry.update();
            }
            InchDrive(21,90,powerlevel/3, "Forward");
            InchDrive(1.95,180,powerlevel/3, "Left");
            OpenClaws();

            InchDrive(11.95,0,powerlevel/3, "right");
            ArmJointMotor.setPower(0.3);
            ArmExtendMotor.setPower(0.3);
            ArmExtendMotor.setTargetPosition(-1);
            while(ArmExtendMotor.getCurrentPosition()<=-100){

            }
            ArmJointMotor.setTargetPosition(50);

            //*
            driverInteruptable(537,-537,-537,537,powerlevel/2, "interuptable left");
            Headlight.setPosition(1);
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
            Headlight.setPosition(0);
            driver(0,0,0,0,powerlevel/2,"Stopping");
            //*/
            ArmJointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            InchDrive(12,270,powerlevel, "Back");
            InchDrive(54,0,powerlevel,"RightOnceMore");
            InchDrive(6,90,powerlevel,"touch forward");
            InchDrive(44,180,powerlevel,"BIG LEFT");
            InchDrive(2,0,powerlevel,"slight right");
            InchDrive(12,270,powerlevel,"back");
            InchDrive(6,180,powerlevel,"left again");
            InchDrive(18,0,powerlevel,"push forward");
            InchDrive(6,270,powerlevel,"go back");

            InchDrive(56,0,powerlevel,"RightOnceMore");
            driver(1975,-1975,1975,-1975, powerlevel, "rotation");
            InchDrive(18,90,powerlevel,"Back");
            CloseClaws();
            SingleMotorDriver(ArmJointMotor, 500,1,"Arm almost ready to fire");
            SingleMotorDriver(ArmJointMotor, 500,-1,"Arm almost ready to fire");


            telemetry.addData("Moves","Done");
            telemetry.update();
            // RESET

        }
    }
}
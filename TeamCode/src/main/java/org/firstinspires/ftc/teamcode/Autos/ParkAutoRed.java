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

import java.util.concurrent.TimeUnit;

@Autonomous(name="ParkZoneAutoRed")

public class ParkAutoRed extends InchAutoParent {

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
        hangArm = hardwareMap.get(DcMotor.class,"hangArm");
        Headlight = hardwareMap.get(Servo.class, "Headlight");
        telemetry.addData("Motors","Got");
        telemetry.update();

        ArmExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmJointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmJointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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
            // Close Claws
            CloseClaws();
            // Moves right, goes forward, then turns left
            InchDrive(26,0,powerlevel, "Right");
            InchDrive(64,90,powerlevel, "Big Forward");
            InchDrive(26,180,powerlevel, "left");


            //moves the arm up
            ArmJointMotor.setTargetPosition(650);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(1);
            // adds telemetry and waits until the encoders
            while(ArmJointMotor.getCurrentPosition()<650){
                telemetry.addData("ExtendPosition: ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("JointPosition: ",ArmJointMotor.getCurrentPosition());
                telemetry.update();

            }
            ArmExtendMotor.setTargetPosition(-2500);
            ArmExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmExtendMotor.setDirection(DcMotor.Direction.FORWARD);
            ArmExtendMotor.setPower(1);
            while(ArmExtendMotor.getCurrentPosition()>-2475){
                telemetry.addData("ExtendPosition: ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("JointPosition: ",ArmJointMotor.getCurrentPosition());
                telemetry.update();

            }

            InchDrive(4,90,powerlevel, "Smol Forward");
            ArmJointMotor.setTargetPosition(775);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(0.4);
            while(ArmJointMotor.getCurrentPosition()<750){
                telemetry.addData("ExtendPosition: ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("JointPosition: ",ArmJointMotor.getCurrentPosition());
                telemetry.update();
            }
            //*/
            TimeUnit.SECONDS.sleep(1);
            OpenClaws();
            TimeUnit.SECONDS.sleep(1);
            CloseClaws();
            ArmJointMotor.setTargetPosition(600);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(1);
            while(ArmJointMotor.getCurrentPosition()>650){
                telemetry.addData("ExtendPosition: ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("JointPosition: ",ArmJointMotor.getCurrentPosition());
                telemetry.update();
            }

            ArmJointMotor.setPower(0.3);
            ArmExtendMotor.setPower(0.6);
            ArmExtendMotor.setTargetPosition(-1);
            while(Math.abs(ArmExtendMotor.getCurrentPosition())>50){
                telemetry.addData("ExtendPosition: ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("JointPosition: ",ArmJointMotor.getCurrentPosition());
                telemetry.update();
            }
            ArmJointMotor.setTargetPosition(50);
            Headlight.setPosition(1);
            /*
            driverInteruptable(517,-517,-517,517,powerlevel/2, "interuptable left");
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
            driver(0,0,0,0,powerlevel/2,"Stopping");
            //*/
            Headlight.setPosition(0);
            ArmJointMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            InchDrive(4, 270,powerlevel,"Back");
            InchDrive(64,0,powerlevel,"RightOnceMore");
            driver(1850, -1850, 1850, -1850, 1, "TURN AROUND!");
            InchDrive(26,270,powerlevel,"Back");
            CloseClaws();
            int pulsepos= ArmJointMotor.getCurrentPosition()+850;
            ArmJointMotor.setTargetPosition(pulsepos);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(1);
            while((ArmJointMotor.getCurrentPosition()<pulsepos-25)||(ArmJointMotor.getCurrentPosition()>pulsepos+25)){
                telemetry.addData("Status","Running...");
                telemetry.update();
            }

            telemetry.addData("Moves","Done");
            telemetry.update();
            // RESET

        }
    }
}
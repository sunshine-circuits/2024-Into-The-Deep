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

@Autonomous(name="BucketZoneAutoGOOD")

public class BucketAutoGood extends InchAutoParent {
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

        myIMU = hardwareMap.get(IMU.class,"imu");

        waitForStart();
        IMU.Parameters myIMUParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        myIMU.initialize(myIMUParameters);

        if (opModeIsActive()) {
            myIMU.resetYaw();
            CloseClaws();

            ArmJointMotor.setTargetPosition(900);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(0.45);
            ArmExtendMotor.setTargetPosition(-2500);
            ArmExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmExtendMotor.setDirection(DcMotor.Direction.FORWARD);
            ArmExtendMotor.setPower(1);

            while((ArmExtendMotor.getCurrentPosition()>-2475)||(ArmJointMotor.getCurrentPosition()<850)){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            InchDrive(18.1107703763,96.3401917459,powerlevel*0.75, "Forward 18 Right 2");
            ArmJointMotor.setTargetPosition(1185);
            while((ArmJointMotor.getCurrentPosition()<1160)){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            ArmJointMotor.setTargetPosition(1215);
            ArmJointMotor.setPower(0.25);
            TimeUnit.MILLISECONDS.sleep(800);
            OpenClaws();
            TimeUnit.MILLISECONDS.sleep(150);
            /*
            ArmJointMotor.setTargetPosition(750);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(1);
            while(ArmJointMotor.getCurrentPosition()>825){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            ArmJointMotor.setPower(0.5);
            ArmExtendMotor.setPower(1);
            ArmExtendMotor.setTargetPosition(100);
            InchDrive(10,270,powerlevel*0.75, "back");
            InchDrive(20.5,0,powerlevel*0.75, "right");
            driverInteruptable(100,-100,-100,100,powerlevel, "Interuptable left");
            Headlight.setPosition(1);
            outerloop:
            while(FRMotor.isBusy()||FLMotor.isBusy()||BRMotor.isBusy()||BLMotor.isBusy()) {
                if (tagProcessor.getDetections().size() > 0) {
                    for (int i = 0; i < tagProcessor.getDetections().size(); i++) {
                        AprilTagDetection tag = tagProcessor.getDetections().get(i);
                        if ((tag.id == 16)||(tag.id == 13)) {
                            break outerloop;
                        }
                    }
                }
            }
            Headlight.setPosition(0);
            //driver(0,0,0,0,powerlevel, "Stopping");
            while(Math.abs(ArmExtendMotor.getCurrentPosition())>150){
                telemetry.addData("ExtendPosition ",Math.abs(ArmExtendMotor.getCurrentPosition()));
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            //turn to the right to try and pick up second sample
            int intcyc=0;
            driverInteruptable(1200, -1200, 1200, -1200, 0.5, "TURN");
            while(5<=(Math.abs(myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+87))){
                intcyc++;
                telemetry.addData("Current Rotation",myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("Checking for",intcyc+" Cycles.");
                telemetry.update();
            }

            //pick it up
            ArmJointMotor.setTargetPosition(2063);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(0.7);
            while(ArmJointMotor.getCurrentPosition()<1500){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            ArmJointMotor.setPower(0.35);
            while(ArmJointMotor.getCurrentPosition()<2038){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            TimeUnit.MILLISECONDS.sleep(500);
            CloseClaws();
            TimeUnit.MILLISECONDS.sleep(400);
            ArmJointMotor.setTargetPosition(825);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(1);
            ArmExtendMotor.setTargetPosition(-2500);
            ArmExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmExtendMotor.setDirection(DcMotor.Direction.FORWARD);
            ArmExtendMotor.setPower(1);
            InchDrive(9,180,powerlevel*0.75, "left");
            while((ArmExtendMotor.getCurrentPosition()>-2475)||(ArmJointMotor.getCurrentPosition()>600)){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            //turn back to face towards the bucket
            driverInteruptable(-1700, 1700, -1700, 1700, 0.5, "Turn Around.");
            while(5<=(Math.abs(myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-45))){
                telemetry.addData("Current Rotation.",myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();
            }
            InchDrive(2,180,powerlevel,"left");
            InchDrive(11,90,powerlevel,"forward");

            ArmJointMotor.setTargetPosition(1163);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(0.4);
            while(ArmJointMotor.getCurrentPosition()<1138){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            //drop the second
            TimeUnit.MILLISECONDS.sleep(750);
            OpenClaws();
            ArmJointMotor.setTargetPosition(750);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(1);
            while(ArmJointMotor.getCurrentPosition()>725){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            ArmJointMotor.setPower(0.5);
            ArmExtendMotor.setPower(1);
            ArmExtendMotor.setTargetPosition(0);
            InchDrive(11, 270,powerlevel,"Back");
            while(Math.abs(ArmExtendMotor.getCurrentPosition())>75){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            ArmJointMotor.setTargetPosition(75);
            driverInteruptable(-1800, 1800, -1800, 1800, 0.8, "TURN");
            while(10<=(Math.abs(myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-170))){
                telemetry.addData("Current Rotation",myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();
                if ((FRMotor.isBusy()||FLMotor.isBusy()||BRMotor.isBusy()||BLMotor.isBusy())==false){
                    break;
                }
            }
            InchDrive(44,180,powerlevel,"Left");
            InchDrive(32,90,powerlevel*1.01,"Back");
            CloseClaws();
            int pulsepos= ArmJointMotor.getCurrentPosition()+1350;
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

            //*/

        }
    }

}
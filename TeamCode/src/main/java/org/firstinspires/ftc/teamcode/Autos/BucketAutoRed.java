package org.firstinspires.ftc.teamcode.Autos;

import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
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

@Autonomous(name="BucketZoneAutoRed")

public class BucketAutoRed extends InchAutoParent {
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
            ArmJointMotor.setTargetPosition(650);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(1);
            ArmExtendMotor.setTargetPosition(-2500);
            ArmExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmExtendMotor.setDirection(DcMotor.Direction.FORWARD);
            ArmExtendMotor.setPower(1);
            while((ArmExtendMotor.getCurrentPosition()>-2475)||(ArmJointMotor.getCurrentPosition()<650)){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            InchDrive(18,90,powerlevel*0.75, "Forward");

            //*
            ArmJointMotor.setTargetPosition(785);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(0.5);
            while(ArmJointMotor.getCurrentPosition()<760){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            ArmJointMotor.setPower(0.3);
            //*/
            TimeUnit.MILLISECONDS.sleep(750);
            OpenClaws();
            ArmJointMotor.setTargetPosition(500);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(1);
            while(ArmJointMotor.getCurrentPosition()>550){
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
            InchDrive(17.5,0,powerlevel*0.75, "right");
            while(Math.abs(ArmExtendMotor.getCurrentPosition())>125){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            //turn to the right to try and pick up second sample

            int intcyc=0;
            driverInteruptable(1000, -1000, 1000, -1000, 0.5, "TURN");
            while(5<=(Math.abs(myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+87))){
                intcyc++;
                telemetry.addData("Current Rotation",myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("Checking for",intcyc+" Cycles.");
                telemetry.update();
            }
            InchDrive(6,0,powerlevel*0.75, "right");
            //pick it up
            ArmJointMotor.setTargetPosition(1320);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(0.7);
            while(ArmJointMotor.getCurrentPosition()<1200){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            ArmJointMotor.setPower(0.3);
            while(ArmJointMotor.getCurrentPosition()<1295){
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
            TimeUnit.MILLISECONDS.sleep(500);
            ArmJointMotor.setTargetPosition(550);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(1);
            ArmExtendMotor.setTargetPosition(-2500);
            ArmExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmExtendMotor.setDirection(DcMotor.Direction.FORWARD);
            ArmExtendMotor.setPower(1);
            InchDrive(6,180,powerlevel*0.75, "right");
            while((ArmExtendMotor.getCurrentPosition()>-2475)||(ArmJointMotor.getCurrentPosition()>575)){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            //turn back to face towards the bucket
            driverInteruptable(-1000, 1000, -1000, 1000, 0.5, "Turn Around.");
            while(5<=(Math.abs(myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-50))){
                telemetry.addData("Current Rotation.",myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();
            }
            driver(0,0,0,0,1,"Stopping.");

            //left move just before we pick things up
            //driver(537,-537,-537,537,powerlevel, "Interuptable left");
            /*
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
            */
            Headlight.setPosition(0);
            //driver(0,0,0,0,powerlevel, "Stopping");
            //*/
            //*
            //InchDrive(3.25,180,powerlevel*0.75, "left");
            InchDrive(16.75,180,powerlevel*0.75, "left");
            InchDrive(1,90,powerlevel, "Forward");

            //*
            ArmJointMotor.setTargetPosition(775);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(0.4);
            while(ArmJointMotor.getCurrentPosition()<750){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            //*/
            //drop the second
            TimeUnit.MILLISECONDS.sleep(750);
            OpenClaws();
            ArmJointMotor.setTargetPosition(500);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(1);
            while(ArmJointMotor.getCurrentPosition()>550){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }

            InchDrive(5, 270,powerlevel,"Back");

            ArmJointMotor.setPower(0.5);
            ArmExtendMotor.setPower(1);
            ArmExtendMotor.setTargetPosition(-1);
            InchDrive(57,0,powerlevel,"RightOnceMore");
            while(Math.abs(ArmExtendMotor.getCurrentPosition())>50){
                telemetry.addData("ExtendPosition ",ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            driverInteruptable(1900, -1900, 1900, -1900, 0.75, "TURN");
            ArmJointMotor.setTargetPosition(50);
            while(7<=(Math.abs(myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180))){
                telemetry.addData("Current Rotation",myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();

            }
            InchDrive(26,90,powerlevel,"Back");
            CloseClaws();
            int pulsepos= ArmJointMotor.getCurrentPosition()+900;
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
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

@Autonomous(name="BucketAutoAprilTagTest")

public class BucketAutoAprilTagTest extends InchAutoParent {
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
        IMU.Parameters myIMUParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,RevHubOrientationOnRobot.UsbFacingDirection.UP));
        myIMU.initialize(myIMUParameters);

        if (opModeIsActive()) {
            myIMU.resetYaw();
            CloseClaws();

            ArmJointMotor.setTargetPosition(900);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(0.5);
            ArmExtendMotor.setTargetPosition(-2500);
            ArmExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmExtendMotor.setDirection(DcMotor.Direction.FORWARD);
            ArmExtendMotor.setPower(1);

            while ((ArmExtendMotor.getCurrentPosition() > -2475) || (ArmJointMotor.getCurrentPosition() < 550)) {
                telemetry.addData("ExtendPosition ", ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ", ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ", ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ", ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ", ArmJointMotor.getMode());
                telemetry.addData("JointTarg ", ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            DistanceDrive(4,18,powerlevel*0.75,"Forward 18 Right 4");
            //InchDrive(18.4390889146,90-12.5288077092,powerlevel*0.75, "Forward 18 Right 4");
            ArmJointMotor.setTargetPosition(790);
            while ((ArmJointMotor.getCurrentPosition() < 750)) {
                telemetry.addData("ExtendPosition ", ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ", ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ", ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ", ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ", ArmJointMotor.getMode());
                telemetry.addData("JointTarg ", ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            ArmJointMotor.setTargetPosition(1215);
            ArmJointMotor.setPower(0.25);
            TimeUnit.MILLISECONDS.sleep(800);
            OpenClaws();
            TimeUnit.MILLISECONDS.sleep(150);
            ArmJointMotor.setTargetPosition(750);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(1);
            while (ArmJointMotor.getCurrentPosition() > 825) {
                telemetry.addData("ExtendPosition ", ArmExtendMotor.getCurrentPosition());
                telemetry.addData("ExtendMode ", ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ", ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ", ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ", ArmJointMotor.getMode());
                telemetry.addData("JointTarg ", ArmJointMotor.getTargetPosition());
                telemetry.update();
            }
            ArmExtendMotor.setPower(1);
            ArmExtendMotor.setTargetPosition(100);
            InchDrive(7, 270, powerlevel, "back");
            InterruptableInchDrive(16.5, 0, powerlevel * 0.7, "right");
            Headlight.setPosition(1);
            outerloop:
            while (FRMotor.isBusy() || FLMotor.isBusy() || BRMotor.isBusy() || BLMotor.isBusy()) {
                if (tagProcessor.getDetections().size() > 0) {
                    for (int i = 0; i < tagProcessor.getDetections().size(); i++) {
                        AprilTagDetection tag = tagProcessor.getDetections().get(i);
                        if (tag.metadata != null) {
                            telemetry.addLine(String.format("\n==== (ID %d) %s", tag.id, tag.metadata.name));
                            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (inch)", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
                            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f (deg)", tag.ftcPose.pitch, tag.ftcPose.roll, tag.ftcPose.yaw));
                            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f (inch, deg, deg)", tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation));
                        } else {
                            telemetry.addLine(String.format("\n==== (ID %d) Unknown", tag.id));
                            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", tag.center.x, tag.center.y));
                        }
                        telemetry.update();
                        if ((tag.id == 16) || (tag.id == 13)) {
                            break outerloop;
                        }
                    }
                }
            }
            driver(0, 0, 0, 0, 1, "Stopping");
            Headlight.setPosition(0);

            /*
            telemetry.addData("IMU Yaw", myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
            TimeUnit.SECONDS.sleep(5);
            // Whoever sees this, I made this code on 2/18, on a tuesday. This is what this code is doing.
            //This is test code to see if the IMU will adjust before detecting the Apriltags and putting it into a variable.
            //The code right below it is commented out because I wanted to make the movement more visible, to see if it would work.
            //1 is turn right, 2 is turn left, 3 is not turn

            int IMUTurnVariable = 3;
            if (myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)<-5) {
                driver(500, -500, 500, -500, 0.5, "TurnLeft");
                IMUTurnVariable = 1;
            } else if (myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)>1) {
                driver(-500, 500, -500, 500, 0.5, "TurnRight");
                IMUTurnVariable = 2;
            }
            if (IMUTurnVariable == 1) {
                while (myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)<-1) {
                    telemetry.addData("IMU Yaw", myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                    telemetry.update();
                }
            } else if (IMUTurnVariable == 2) {
                while (myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)>1) {
                    telemetry.addData("IMU Yaw", myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                    telemetry.update();
                }
            }
            driver(0, 0, 0, 0, 0.5, "Stopping");
            //*/
            double xAprilTag=-3.141592659;
            double yAprilTag=-3.141592659;
            // ARPIL TAG STUFF
            for (int i = 0; i < tagProcessor.getDetections().size(); i++) {
                AprilTagDetection tag = tagProcessor.getDetections().get(i);
                if (tag.metadata != null) {
                    xAprilTag = tag.ftcPose.x;
                    yAprilTag = tag.ftcPose.y;
                    telemetry.addLine(String.format("\n==== (ID %d) %s", tag.id, tag.metadata.name));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (inch)", tag.ftcPose.x, tag.ftcPose.y, tag.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f (deg)", tag.ftcPose.pitch, tag.ftcPose.roll, tag.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f (inch, deg, deg)", tag.ftcPose.range, tag.ftcPose.bearing, tag.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", tag.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", tag.center.x, tag.center.y));
                }
                telemetry.addData("IMU Yaw", myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("IMU Pitch", myIMU.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
                telemetry.addData("IMU Roll", myIMU.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
                telemetry.update();
            }

            while(Math.abs(ArmExtendMotor.getCurrentPosition())>175){
                /*
                telemetry.addData("ExtendPosition ",Math.abs(ArmExtendMotor.getCurrentPosition()));
                telemetry.addData("ExtendMode ",ArmExtendMotor.getMode());
                telemetry.addData("ExtendTarg ",ArmExtendMotor.getTargetPosition());
                telemetry.addData("JointPosition ",ArmJointMotor.getCurrentPosition());
                telemetry.addData("JointMode ",ArmJointMotor.getMode());
                telemetry.addData("JointTarg ",ArmJointMotor.getTargetPosition());
                telemetry.update();
                */
            }
            //turn to the right to try and pick up second sample
            int intcyc=0;
            driverInteruptable(13000, -13000, 13000, -13000, 0.5, "TURN");
            while(2<=(Math.abs(myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+88))){
                /*
                intcyc++;
                telemetry.addData("Current Rotation",myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("Checking for",intcyc+" Cycles.");
                telemetry.update();
                */
            }
            driver(0, 0, 0, 0, 1, "Stopping");
            telemetry.addData("XAprilTag", xAprilTag);
            telemetry.addData("YAprilTag", yAprilTag);
            telemetry.update();
            InchDrive(xAprilTag+0.75,90,powerlevel*0.8, "move backwards");
            ArmJointMotor.setTargetPosition(2080);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(0.75);
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
            while(ArmJointMotor.getCurrentPosition()<2055){
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
            ArmJointMotor.setPower(1);ArmExtendMotor.setTargetPosition(-2500);
            ArmExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmExtendMotor.setDirection(DcMotor.Direction.FORWARD);
            ArmExtendMotor.setPower(1);
            InchDrive(9,180,powerlevel, "left");
            while((ArmExtendMotor.getCurrentPosition()>-2475)||(ArmJointMotor.getCurrentPosition()<775)){
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
            while(4<=(Math.abs(myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-45))){
                telemetry.addData("Current Rotation.",myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();
            }
            DistanceDrive(7,7,powerlevel,"GETTHEBUCKET");
            ArmJointMotor.setTargetPosition(1150);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(0.7);
            while(ArmJointMotor.getCurrentPosition()<1125){
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
            ArmJointMotor.setPower(1);
            ArmExtendMotor.setPower(1);
            ArmExtendMotor.setTargetPosition(0);
            InchDrive(11, 270,powerlevel,"Back");
            ArmJointMotor.setTargetPosition(75);
            driverInteruptable(-1800, 1800, -1800, 1800, 0.8, "TURN");
            while(10<=(Math.abs(myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-175))){
                telemetry.addData("Current Rotation",myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();
                if ((FRMotor.isBusy()||FLMotor.isBusy()||BRMotor.isBusy()||BLMotor.isBusy())==false){
                    break;
                }
            }
            CloseClaws();
            SloppyInchDrive(40,180,1,"Left");
            SloppyInchDrive(32,90,1,"Back");
            int pulsepos= ArmJointMotor.getCurrentPosition()+1350;
            ArmJointMotor.setTargetPosition(pulsepos);
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ArmJointMotor.setPower(1);
            while((ArmJointMotor.getCurrentPosition()<pulsepos-30)||(ArmJointMotor.getCurrentPosition()>pulsepos+30)){
                telemetry.addData("Status","Running...");
                telemetry.update();
            }
            telemetry.addData("Moves","Done");
            telemetry.update();
        }
    }

}
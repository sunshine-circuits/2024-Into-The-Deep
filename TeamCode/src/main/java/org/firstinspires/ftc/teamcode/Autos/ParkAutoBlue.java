package org.firstinspires.ftc.teamcode.Autos;

import android.util.Size;

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

@Autonomous(name="ParkZoneAutoBlue")

public class ParkAutoBlue extends InchAutoParent {

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
        myIMU = hardwareMap.get(IMU.class, "myIMU");
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
            InchDrive(75,90,powerlevel, "Big Forward");
            //InchDrive(26.5,180,powerlevel, "left");
            myIMU.resetYaw();
            driverInteruptable(-1000,1000,-1000,1000,1,"FalseRight");
            while(!(Math.abs(myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-45)<1)){
                telemetry.addData("turning",myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();
                if(Math.abs(myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)-45)<1){
                    break;
                }
            }

            //attempted rotation drive code RotationDrive(0, 0, 45, 0.5, "doing rotation");
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

            InchDrive(6,90,powerlevel, "Smol Forward");
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
                        if (tag.id == 13) {

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
            InchDrive(6, 270,powerlevel,"Back");
            InchDrive(55,0,powerlevel,"RightOnceMore");
            InchDrive(26,270,powerlevel,"Back");
            CloseClaws();
            hangArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hangArm.setPower(0.5);
            TimeUnit.MILLISECONDS.sleep(1450);

            hangArm.setPower(0);

            telemetry.addData("Moves","Done");
            telemetry.update();
            // RESET

        }
    }
}
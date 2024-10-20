package org.firstinspires.ftc.teamcode.utils;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.Driver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
public class Driver extends LinearOpMode{
    // Hardware declarations
    public DcMotor frontRightWheel;
    public DcMotor frontLeftWheel;
    public DcMotor backRightWheel;
    public DcMotor backLeftWheel;
    public DcMotor ArmJointMotor;
    public DcMotor ArmExtendMotor;
    public CRServo LeftServo;
    public CRServo RightServo;
    public double speedMultiplier =1;
    public Coordinate myLocation;
    double minArmExtensionPosition;
    double maxArmExtensionPosition;
    double frontRightTargetPow;
    double frontLeftTargetPow;
    double backRightTargetPow;
    double backLeftTargetPow;
    double ArmJointTargetPow;
    double ArmExtendTargetPow;
    boolean ExtPos;
    boolean ArmPos;
    double LeftServoPower;
    double RightServoPower;
    DcMotor.RunMode defaultMode;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    boolean triggerdown;

    public double[] CameraCoordinates(int tagID, double Yaw, double Xcamera, double Ycamera){
        double distance = Math.sqrt((Xcamera*Xcamera)+(Ycamera*Ycamera));
        double Xreltrue = Math.sin(Yaw)*distance;
        double Yreltrue = Math.cos(Yaw)*distance;

        double TagX = Grid.AreasOfInterest.valueOf("TAG"+tagID).coord.xPosition;
        double TagY = Grid.AreasOfInterest.valueOf("TAG"+tagID).coord.yPosition;

        double XtrueCord = TagX-Xreltrue;
        double YtrueCord = TagY-Yreltrue;
        return new double[]{XtrueCord, YtrueCord};
    }
    protected void initOpMode() {
        frontRightWheel = hardwareMap.get(DcMotor.class,"FRMotor");
        frontLeftWheel = hardwareMap.get(DcMotor.class,"FLMotor");
        backRightWheel = hardwareMap.get(DcMotor.class,"BRMotor");
        backLeftWheel = hardwareMap.get(DcMotor.class,"BLMotor");
        ArmJointMotor = hardwareMap.get(DcMotor.class,"ArmJointMotor");
        ArmExtendMotor = hardwareMap.get(DcMotor.class, "ArmExtendMotor");
        LeftServo = hardwareMap.get(CRServo.class, "LeftServo");
        RightServo = hardwareMap.get(CRServo.class, "RightServo");

        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmJointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmJointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myLocation = new Coordinate(12,24,6,0);
    }

    public void basicMotion(){

    }

    public void setupDrivingVariables(){
        minArmExtensionPosition = ArmExtendMotor.getCurrentPosition();
        maxArmExtensionPosition = minArmExtensionPosition+1000;
        frontRightTargetPow=0;
        frontLeftTargetPow=0;
        backRightTargetPow=0;
        backLeftTargetPow=0;
        ArmJointTargetPow=0;
        ArmExtendTargetPow=0;
        ExtPos=true;
        ArmPos=true;
        LeftServoPower=0;
        RightServoPower=0;
        defaultMode = ArmExtendMotor.getMode();
    }
    public void setupAprilTags(){
        tagProcessor = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagID(true).setDrawTagOutline(true).setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11).setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()).build();
        visionPortal= new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam")).setCameraResolution(new Size(640,480)).build();
    }

    public void BasicMotionHandler(){
        //Motors
        //movement for forward&back,      left&right            Rotation
        frontRightTargetPow = 0 + gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        frontRightWheel.setPower(speedMultiplier*frontRightTargetPow);
        telemetry.addData("Front Right TP", frontRightTargetPow);
        telemetry.addData("Front Right CP", frontRightWheel.getPower());

        frontLeftTargetPow = 0 - gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
        frontLeftWheel.setPower(speedMultiplier*frontLeftTargetPow);
        telemetry.addData("Front Left TP", frontLeftTargetPow);
        telemetry.addData("Front Left CP", frontLeftWheel.getPower());

        backRightTargetPow = 0 + gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        backRightWheel.setPower(speedMultiplier*backRightTargetPow);
        telemetry.addData("Back Right TP", backRightTargetPow);
        telemetry.addData("Back Right CP", backRightWheel.getPower());

        backLeftTargetPow = 0 - gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        backLeftWheel.setPower(speedMultiplier*backLeftTargetPow);
        telemetry.addData("Back Left TP", backLeftTargetPow);
        telemetry.addData("Back Left CP", backLeftWheel.getPower());
    }
    public void updateSpeedControl(){
        //Speed Multiplier
        if (gamepad1.right_trigger>=0.5){
            if (triggerdown==false){
                if (speedMultiplier < 1) {
                    speedMultiplier = speedMultiplier + 0.1;
                    triggerdown=true;
                }
            }
        }
        else if (gamepad1.left_trigger>=0.5){
            if(triggerdown==false) {
                if (speedMultiplier > 0.1) {
                    speedMultiplier = speedMultiplier - 0.1;
                    triggerdown=true;
                }
            }
        }else{
            triggerdown=false;
        }
        telemetry.addData("Speed Multiplier:", speedMultiplier);
    }

    public void ArmExtendHandler(){
        //arm code
        if (Math.abs(gamepad2.right_stick_y)>=0.1){
            ArmExtendMotor.setMode(defaultMode);
            ArmExtendTargetPow=gamepad2.right_stick_y;
            ExtPos=true;
        } else{
            ArmExtendTargetPow=1;
            if (ExtPos==true) {
                ArmExtendMotor.setTargetPosition(ArmExtendMotor.getCurrentPosition());
                ExtPos=false;
            }
            ArmExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        telemetry.addData("ArmPow", ArmExtendTargetPow);
        ArmExtendMotor.setPower(ArmExtendTargetPow);
    }

    public void ArmJointHandler(){
        if (gamepad2.dpad_right){
            ArmJointMotor.setMode(defaultMode);
            ArmJointTargetPow=0.5;
            ArmPos=true;
        }
        else if (gamepad2.dpad_left){
            ArmJointMotor.setMode(defaultMode);
            ArmJointTargetPow=-0.5;
            ArmPos=true;
        } else{
            ArmJointTargetPow=1;
            if (ArmPos==true) {
                ArmJointMotor.setTargetPosition(ArmJointMotor.getCurrentPosition());
                ArmPos=false;
            }
            ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        telemetry.addData("JoinPow", ArmJointTargetPow);
        ArmJointMotor.setPower(ArmJointTargetPow);
    }
    public void ServoHandler(){
        //Servo Code
        if (gamepad2.x){
            LeftServoPower=0.25;
            RightServoPower=-0.25;
        }
        else if (gamepad2.y){
            LeftServoPower=-0.25;
            RightServoPower=0.25;
        } else{
            LeftServoPower=0;
            RightServoPower=0;
        }
        telemetry.addData("Right Servo: ", RightServo.getDirection());
        telemetry.addData("Left Servo: ", LeftServo.getDirection());
        telemetry.addData("armpow", LeftServoPower);

        LeftServo.setPower(LeftServoPower);
        RightServo.setPower(RightServoPower);
    }
    public void AprilTagHandler(){
        boolean april=false;
        //Camera Processing--- Cordinates based on aprilTags
        if (!tagProcessor.getDetections().isEmpty()) {
            for (int i = 0; i < tagProcessor.getDetections().size(); i++) {
                april=true;
                AprilTagDetection tag = tagProcessor.getDetections().get(i);
                try {
                    double[] mycords = CameraCoordinates(tag.id, tag.ftcPose.yaw, tag.ftcPose.x, tag.ftcPose.y);
                    myLocation.xPosition = mycords[0];
                    myLocation.yPosition = mycords[1];
                    telemetry.addData("myXbytag " + tag.id, myLocation.xPosition);
                    telemetry.addData("myYbytag " + tag.id, myLocation.yPosition);
                } catch(Exception e){
                    telemetry.addData("Unidentified tag","Unidentified tag" );
                }
            }
        }
        telemetry.addData("CanSeeAprilTag",april);
    }
    @Override
    public void runOpMode() throws InterruptedException
    {
        initOpMode();
        setupAprilTags();
        setupDrivingVariables();
        waitForStart();

        while(opModeIsActive())
        {
            BasicMotionHandler();
            updateSpeedControl();
            ArmExtendHandler();
            ArmJointHandler();
            ServoHandler();
            AprilTagHandler();
            telemetry.update();
        }
    }
}

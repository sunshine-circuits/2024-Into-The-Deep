package org.firstinspires.ftc.teamcode.TeleOp;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.utils.Coordinate;
import org.firstinspires.ftc.teamcode.utils.Grid;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name="Robot Orienter")
public class OrientOp extends LinearOpMode
{
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
    private Coordinate myLocation;

    //calculates the position of the camera in our cordinate system based on the position of the april tag.
    //our cordinate system is based on inches, and begins at the blue corner without the bucket.
    public double[] CameraCoordinates(int tagID, double Yaw, double Xcamera, double Ycamera){
        double distance = Math.sqrt((Xcamera*Xcamera)+(Ycamera*Ycamera));
        double Xreltrue = Math.sin(Yaw)*distance;
        double Yreltrue = Math.cos(Yaw)*distance;

        double TagX = Grid.valueOf("TAG"+tagID).coord.xPosition;
        double TagY = Grid.valueOf("TAG"+tagID).coord.yPosition;

        double XtrueCord = TagX-Xreltrue;
        double YtrueCord = TagY-Yreltrue;
        return new double[]{XtrueCord, YtrueCord};
    }
    public void driver(int FR, int FL, int BR, int BL, double pow, String telem){
        int FRtargetpos;
        int FLtargetpos;
        int BRtargetpos;
        int BLtargetpos;

        telemetry.addData("Variables "+telem,"Declared");
        telemetry.update();

        FRtargetpos=frontRightWheel.getCurrentPosition()-(FR);
        FLtargetpos=frontLeftWheel.getCurrentPosition()+(FL);
        BRtargetpos=backRightWheel.getCurrentPosition()-(BR);
        BLtargetpos=backLeftWheel.getCurrentPosition()+(BL);

        telemetry.addData("MotorData "+telem,"Snatched");
        telemetry.update();

        frontRightWheel.setTargetPosition(FRtargetpos);
        frontLeftWheel.setTargetPosition(FLtargetpos);
        backRightWheel.setTargetPosition(BRtargetpos);
        backLeftWheel.setTargetPosition(BLtargetpos);

        telemetry.addData("TargetPos "+telem,"Declared");
        telemetry.addData("TargetPosFR ",FRtargetpos);
        telemetry.addData("TargetPosFL ",FLtargetpos);
        telemetry.addData("TargetPosBR ",BRtargetpos);
        telemetry.addData("TargetPosBL ",BLtargetpos);
        telemetry.update();

        frontRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Running "+telem,"Happening");
        telemetry.update();

        frontRightWheel.setPower(pow);
        frontLeftWheel.setPower(pow);
        backRightWheel.setPower(pow);
        backLeftWheel.setPower(pow);

        telemetry.addData("Power "+telem,"Set");
        telemetry.update();

        telemetry.addData("While "+telem,"Started");
        telemetry.update();

        while (opModeIsActive()&&
                (
                        frontRightWheel.isBusy()||frontLeftWheel.isBusy()||backRightWheel.isBusy()||backLeftWheel.isBusy()
                )
        )
        {
            telemetry.addData("CurrentPosFR",frontRightWheel.getCurrentPosition());
            telemetry.addData("CurrentPosFL",frontLeftWheel.getCurrentPosition());
            telemetry.addData("CurrentPosBR",backRightWheel.getCurrentPosition());
            telemetry.addData("CurrentPosBL",backLeftWheel.getCurrentPosition());

            telemetry.addData("TargetPosFR ",FRtargetpos);
            telemetry.addData("TargetPosFL ",FLtargetpos);
            telemetry.addData("TargetPosBR ",BRtargetpos);
            telemetry.addData("TargetPosBL ",BLtargetpos);
            telemetry.update();
        }

        telemetry.addData("While "+telem,"Done");
        telemetry.update();

        frontRightWheel.setPower(0);
        frontLeftWheel.setPower(0);
        backRightWheel.setPower(0);
        backLeftWheel.setPower(0);

        telemetry.addData("Power","Zero");
        telemetry.update();
    }

    private void initOpMode() {
        frontRightWheel = hardwareMap.get(DcMotor.class,"FRMotor");
        frontLeftWheel = hardwareMap.get(DcMotor.class,"FLMotor");
        backRightWheel = hardwareMap.get(DcMotor.class,"BRMotor");
        backLeftWheel = hardwareMap.get(DcMotor.class,"BLMotor");
        ArmJointMotor = hardwareMap.get(DcMotor.class,"ArmJointMotor");
        ArmExtendMotor = hardwareMap.get(DcMotor.class, "ArmExtendMotor");
        LeftServo = hardwareMap.get(CRServo.class, "LeftServo");
        RightServo = hardwareMap.get(CRServo.class, "RightServo");

        telemetry.addData("Status","Intialized");
        telemetry.addData("Handicap", speedMultiplier);
        telemetry.update();

        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmJointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmJointMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        myLocation = new Coordinate(12,24,6,0);
    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initOpMode();
        DcMotor.RunMode defaultMode = ArmExtendMotor.getMode();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagID(true).setDrawTagOutline(true).setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11).setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()).build();
        VisionPortal visionPortal= new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam")).setCameraResolution(new Size(640,480)).build();

        waitForStart();
        double frontRightTargetPow=0;
        double frontLeftTargetPow=0;
        double backRightTargetPow=0;
        double backLeftTargetPow=0;
        double ArmJointTargetPow=0;
        double ArmExtendTargetPow=0;
        boolean ExtPos=true;
        boolean ArmPos=true;
        double LeftServoPower=0;
        double RightServoPower=0;

        while(opModeIsActive())
        {
            telemetry.addData("Status","Running");
            telemetry.update();
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


            //Speed Multiplier
            if (gamepad1.right_trigger>=0.5){
                if (speedMultiplier <1){
                    speedMultiplier = speedMultiplier +0.1;
                }
                while (gamepad1.right_trigger>=0.5){

                }
            }
            else if (gamepad1.left_trigger>=0.5){
                if (speedMultiplier >0.1){
                    speedMultiplier = speedMultiplier -0.1;
                }
                while (gamepad1.left_trigger>=0.5){

                }
            }
            telemetry.addData("Speed Multiplier:", speedMultiplier);

            //arm code
            if (gamepad1.dpad_up){
                ArmExtendMotor.setMode(defaultMode);
                ArmExtendTargetPow=1;
                ExtPos=true;
            }
            else if (gamepad1.dpad_down){
                ArmExtendMotor.setMode(defaultMode);
                ArmExtendTargetPow=-1;
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

            if (gamepad1.dpad_right){
                ArmJointMotor.setMode(defaultMode);
                ArmJointTargetPow=0.5;
                ArmPos=true;
            }
            else if (gamepad1.dpad_left){
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
            //Servo Code
            if (gamepad1.x){
                LeftServoPower=0.25;
                RightServoPower=-0.25;

            }
            else if (gamepad1.y){
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
                        myLocation.yawRotation = tag.ftcPose.yaw;
                        telemetry.addData("myXbytag " + tag.id, myLocation.xPosition);
                        telemetry.addData("myYbytag " + tag.id, myLocation.yPosition);
                    } catch(Exception e){
                        telemetry.addData("Unidentified tag","Unidentified tag" );
                    }
                }
            }
            telemetry.addData("CanSeeAprilTag",april);

            //930 is approximately 90 degrees
            if(gamepad1.a){
                int rotby = (int)(((myLocation.yawRotation+90)/90)*930);
                driver(rotby, -rotby, rotby, -rotby, 0.5, "Funky");
            }else if(gamepad1.b){
                int rotby = (int)(((myLocation.yawRotation+90)/90)*930);
                driver(-rotby, rotby, -rotby, rotby, 0.5, "Funky");
            }




            telemetry.update();
        }
    }

}
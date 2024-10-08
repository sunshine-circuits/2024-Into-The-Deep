package org.firstinspires.ftc.teamcode.Autos;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="FirstAuto")

public class FirstAuto extends LinearOpMode{
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    //    @Override
    public void driver(int FR, int FL, int BR, int BL, double pow, String telem){
        int FRtargetpos;
        int FLtargetpos;
        int BRtargetpos;
        int BLtargetpos;

        telemetry.addData("Variables "+telem,"Declared");
        telemetry.update();

        FRtargetpos=FRMotor.getCurrentPosition()-(FR);
        FLtargetpos=FLMotor.getCurrentPosition()+(FL);
        BRtargetpos=BRMotor.getCurrentPosition()-(BR);
        BLtargetpos=BLMotor.getCurrentPosition()+(BL);

        telemetry.addData("MotorData "+telem,"Snatched");
        telemetry.update();

        FRMotor.setTargetPosition(FRtargetpos);
        FLMotor.setTargetPosition(FLtargetpos);
        BRMotor.setTargetPosition(BRtargetpos);
        BLMotor.setTargetPosition(BLtargetpos);

        telemetry.addData("TargetPos "+telem,"Declared");
        telemetry.addData("TargetPosFR ",FRtargetpos);
        telemetry.addData("TargetPosFL ",FLtargetpos);
        telemetry.addData("TargetPosBR ",BRtargetpos);
        telemetry.addData("TargetPosBL ",BLtargetpos);
        telemetry.update();

        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Running "+telem,"Happening");
        telemetry.update();

        FRMotor.setPower(pow);
        FLMotor.setPower(pow);
        BRMotor.setPower(pow);
        BLMotor.setPower(pow);

        telemetry.addData("Power "+telem,"Set");
        telemetry.update();

        telemetry.addData("While "+telem,"Started");
        telemetry.update();

        while (opModeIsActive()&&
                (
                        FRMotor.isBusy()||FLMotor.isBusy()||BRMotor.isBusy()||BLMotor.isBusy()
                )
        )
        {
            telemetry.addData("CurrentPosFR",FRMotor.getCurrentPosition());
            telemetry.addData("CurrentPosFL",FLMotor.getCurrentPosition());
            telemetry.addData("CurrentPosBR",BRMotor.getCurrentPosition());
            telemetry.addData("CurrentPosBL",BLMotor.getCurrentPosition());

            telemetry.addData("TargetPosFR ",FRtargetpos);
            telemetry.addData("TargetPosFL ",FLtargetpos);
            telemetry.addData("TargetPosBR ",BRtargetpos);
            telemetry.addData("TargetPosBL ",BLtargetpos);
            telemetry.update();
        }

        telemetry.addData("While "+telem,"Done");
        telemetry.update();

        FRMotor.setPower(0);
        FLMotor.setPower(0);
        BRMotor.setPower(0);
        BLMotor.setPower(0);

        telemetry.addData("Power","Zero");
        telemetry.update();
    }

    public void driverInteruptable(int FR, int FL, int BR, int BL, double pow, String telem){
        int FRtargetpos;
        int FLtargetpos;
        int BRtargetpos;
        int BLtargetpos;

        telemetry.addData("Variables "+telem,"Declared");
        telemetry.update();

        FRtargetpos=FRMotor.getCurrentPosition()-(FR);
        FLtargetpos=FLMotor.getCurrentPosition()+(FL);
        BRtargetpos=BRMotor.getCurrentPosition()-(BR);
        BLtargetpos=BLMotor.getCurrentPosition()+(BL);

        telemetry.addData("MotorData "+telem,"Snatched");
        telemetry.update();

        FRMotor.setTargetPosition(FRtargetpos);
        FLMotor.setTargetPosition(FLtargetpos);
        BRMotor.setTargetPosition(BRtargetpos);
        BLMotor.setTargetPosition(BLtargetpos);

        telemetry.addData("TargetPos "+telem,"Declared");
        telemetry.addData("TargetPosFR ",FRtargetpos);
        telemetry.addData("TargetPosFL ",FLtargetpos);
        telemetry.addData("TargetPosBR ",BRtargetpos);
        telemetry.addData("TargetPosBL ",BLtargetpos);
        telemetry.update();

        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Running "+telem,"Happening");
        telemetry.update();

        FRMotor.setPower(pow);
        FLMotor.setPower(pow);
        BRMotor.setPower(pow);
        BLMotor.setPower(pow);

        telemetry.addData("Power "+telem,"Set");
        telemetry.update();

        telemetry.addData("Disuptable "+telem,"Started");
        telemetry.update();
    }

    public void DriveUpt(){

    }

    //*/
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("Motors","Grabbing");
        telemetry.update();
        FRMotor = hardwareMap.get(DcMotor.class,"FRMotor");
        FLMotor = hardwareMap.get(DcMotor.class,"FLMotor");
        BRMotor = hardwareMap.get(DcMotor.class,"BRMotor");
        BLMotor = hardwareMap.get(DcMotor.class,"BLMotor");
        telemetry.addData("Motors","Got");
        telemetry.update();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder().setDrawAxes(true).setDrawCubeProjection(true).setDrawTagID(true).setDrawTagOutline(true).setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11).setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary()).build();
        VisionPortal visionPortal= new VisionPortal.Builder().addProcessor(tagProcessor).setCamera(hardwareMap.get(WebcamName.class, "Webcam")).setCameraResolution(new Size(640,480)).build();
        waitForStart();
        if (opModeIsActive()) {
            driver(-1,1,1,-1,0.5, "Right");
            driver(1,1,1,1,0.5, "Big Forward");
            //outerloop:
            //while (true) {
            driverInteruptable(1,-1,-1,1,0.25, "interuptable left");
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
            driver(0,0,0,0,0.5,"Stopping");
            driver(-1,-1,-1,-1,0.5,"Back");
            driver(-1,1,1,-1,0.5, "BIG RIGHT");
            driver(-1,-1,-1,-1,0.5,"Back");
            //}


            telemetry.addData("Moves","Done");
            telemetry.update();
            // RESET
        }
    }
}
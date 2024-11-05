package org.firstinspires.ftc.teamcode.Autos;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="FirstInchAuto")

public class FirstAuto extends LinearOpMode{
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor ArmJointMotor;
    private Servo Headlight;
    private TouchSensor ArmHomeSensor;
    private double powerlevel=0.85;

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
            telemetry.addData(telem+" CurrentPosFR",FRMotor.getCurrentPosition());
            telemetry.addData(telem+" CurrentPosFL",FLMotor.getCurrentPosition());
            telemetry.addData(telem+" CurrentPosBR",BRMotor.getCurrentPosition());
            telemetry.addData(telem+" CurrentPosBL",BLMotor.getCurrentPosition());

            telemetry.addData(telem+" TargetPosFR ",FRtargetpos);
            telemetry.addData(telem+" TargetPosFL ",FLtargetpos);
            telemetry.addData(telem+" TargetPosBR ",BRtargetpos);
            telemetry.addData(telem+" TargetPosBL ",BLtargetpos);
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
    public void InchDrive(double distance, double angle, double pow, String telem){
        double pulses = InchesToPulses(distance);
        double diameterOfWheels = 104.0 / 25.4;
        double pulsesInX = ((Math.cos((angle*Math.PI)/180.0) * distance) / (Math.PI * diameterOfWheels)) * 537.7;
        double pulsesInY = ((Math.sin((angle*Math.PI)/180.0) * distance) / (Math.PI * diameterOfWheels)) * 537.7;
        driver((int)(pulsesInY - pulsesInX), (int)(pulsesInY + pulsesInX), (int)(pulsesInY + pulsesInX), (int)(pulsesInY - pulsesInX), pow, telem);
        //FR, FL, BR, BL
    }
    public void RotationDrive(double distance, double angleDrive, double angle, double pow, String telem){
        /*
        * The angleDrive variable is the angle that the robot is driving in(according to orientation)
        * The distance variable is the distance being driven(will scale the the x, y, and z)
        * the angle variable is the angle that the robot will turn in while driving*/
        double pulses = InchesToPulses(distance);
        double diameterOfWheels = 104 / 25.4;
        double rotationsInX = ((Math.cos(angleDrive*Math.PI/180) * distance) / (Math.PI * diameterOfWheels));
        double rotationsInY = ((Math.sin(angleDrive*Math.PI/180) * distance) / (Math.PI * diameterOfWheels));
        driver((int)((rotationsInY - rotationsInX - angle/360) * 537.7), (int)((rotationsInY + rotationsInX + angle/360) * 537.7), (int)((rotationsInY + rotationsInX - angle/360) * 537.7), (int)((rotationsInY - rotationsInX + angle/360) * 537.7), pow, telem);
        //FR, FL, BR, BL
    }
    public double InchesToPulses(double Inches){
        return 537.7*(Inches/((104*Math.PI)/25.4));
    }
    double myrot;
    public void RotateDegrees(double angle, double pow, String telem){
        double pulses = RotationsToPulses(angle-myrot);
        int pulsesInt = (int)pulses;
        driver(-pulsesInt,pulsesInt,-pulsesInt,pulsesInt,pow,telem);
    }
    public double RotationsToPulses(double degrees){
        return degrees*(((590/360)*537.7)/90);
    }

    public void SingleMotorDriver(DcMotor OpMotor, int dist, double pow, String telem){
        int OpPos;
        telemetry.addData("Variables "+telem,"Declared");
        telemetry.update();

        OpPos=OpMotor.getCurrentPosition()+dist;

        telemetry.addData("MotorData "+telem,"Snatched");
        telemetry.update();

        OpMotor.setTargetPosition(OpPos);

        telemetry.addData("TargetPos "+telem,"Declared");
        telemetry.addData("TargetPosFR ",OpPos);
        telemetry.update();

        OpMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Running "+telem,"Happening");
        telemetry.update();

        OpMotor.setPower(pow);

        telemetry.addData("Power "+telem,"Set");
        telemetry.update();

        telemetry.addData("While "+telem,"Started");
        telemetry.update();

        while (opModeIsActive()&&(OpMotor.isBusy()))
        {
            telemetry.addData("CurrentPos",OpMotor.getCurrentPosition());

            telemetry.addData("TargetPos ",OpPos);
            telemetry.update();
        }

        telemetry.addData("While "+telem,"Done");
        telemetry.update();

        OpMotor.setPower(0);

        telemetry.addData("Power","Zero");
        telemetry.update();
    }

    public void InteruptableSingleMotorDriver(DcMotor OpMotor, int dist, double pow, String telem){
        int OpPos;
        telemetry.addData("Variables "+telem,"Declared");
        telemetry.update();

        OpPos=OpMotor.getCurrentPosition()+dist;

        telemetry.addData("MotorData "+telem,"Snatched");
        telemetry.update();

        OpMotor.setTargetPosition(OpPos);

        telemetry.addData("TargetPos "+telem,"Declared");
        telemetry.addData("TargetPosFR ",OpPos);
        telemetry.update();

        OpMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Running "+telem,"Happening");
        telemetry.update();

        OpMotor.setPower(pow);

        telemetry.addData("Power "+telem,"Set");
        telemetry.update();

        telemetry.addData("Disruptable "+telem,"Started");
        telemetry.update();
    }


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
        ArmHomeSensor = hardwareMap.get(TouchSensor.class, "ArmHomeTouchSensor");
        telemetry.addData("Motors","Got");
        telemetry.update();

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
        Headlight.setPosition(1);
        if (opModeIsActive()) {
            InchDrive(2,0,powerlevel, "Right");
            InchDrive(82.5,90,powerlevel, "Big Forward");
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
            InchDrive(54,0,powerlevel,"RightOnceMore");
            SingleMotorDriver(ArmJointMotor, 5000,powerlevel,"Arm almost ready to fire");
            InchDrive(15,-90,powerlevel,"Back");
            SingleMotorDriver(ArmJointMotor, -5000,powerlevel,"Arm in position");

            telemetry.addData("Moves","Done");
            telemetry.update();
            // RESET

        }
    }
}
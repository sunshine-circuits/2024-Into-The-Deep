package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class InchAutoParent extends LinearOpMode {
    protected DcMotor BLMotor;
    protected DcMotor BRMotor;
    protected DcMotor FLMotor;
    protected DcMotor FRMotor;
    protected DcMotor ArmJointMotor;
    protected Servo Headlight;
    protected TouchSensor ArmHomeSensor;
    protected double powerlevel=0.75;

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

    @Override
    public void runOpMode() throws InterruptedException {

    }
}

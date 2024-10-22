package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoDriver {

    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor rearRightMotor;
    private DcMotor rearLeftMotor;

    private Coordinate myLocation;

    public AutoDriver(RobotConfig.Config config){
        this.frontRightMotor = (DcMotor)config.get("FRMotor");
        this.rearRightMotor = (DcMotor)config.get("BRMotor");
        this.frontLeftMotor = (DcMotor)config.get("FLMotor");
        this.rearLeftMotor = (DcMotor)config.get("BLMotor");
    }

    public void drive(int FR, int FL, int BR, int BL, double pow){
        int FRtargetpos=frontRightMotor.getCurrentPosition()-(FR);
        int FLtargetpos=frontLeftMotor.getCurrentPosition()+(FL);
        int BRtargetpos=rearRightMotor.getCurrentPosition()-(BR);
        int BLtargetpos=rearLeftMotor.getCurrentPosition()+(BL);

        frontRightMotor.setTargetPosition(FRtargetpos);
        frontLeftMotor.setTargetPosition(FLtargetpos);
        rearRightMotor.setTargetPosition(BRtargetpos);
        rearLeftMotor.setTargetPosition(BLtargetpos);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
    }

    public void driveDistance(double distance, double angle, double pow){
        double pulses = inchesToPulses(distance);
        int pulsesInt = (int)pulses;
        drive(pulsesInt,pulsesInt,pulsesInt,pulsesInt,pow);
    }
    private double inchesToPulses(double Inches){
        return 537.7*(Inches/((104*Math.PI)/25.4));
    }

    public void rotate(double angle, double pow){
        double pulses = rotationsToPulses(angle-myLocation.yawRotation);
        int pulsesInt = (int)pulses;
        drive(-pulsesInt,pulsesInt,-pulsesInt,pulsesInt,pow);
    }
    private double rotationsToPulses(double degrees){
        return ((degrees/90)*930);
    }

    //TODO: Add AprilTag processing here and also add basic flow logic for pathing
}

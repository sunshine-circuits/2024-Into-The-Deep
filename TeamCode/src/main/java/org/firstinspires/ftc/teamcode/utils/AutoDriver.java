package org.firstinspires.ftc.teamcode.utils;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class AutoDriver {
    private DcMotor ArmMotor0;
    private DcMotor ArmMotor1;
    public CRServo Servo0;
    public CRServo Servo1;
    private DcMotor ArmJointMotor;
    private double angle;

    public double Pulses(double Inches){
        double pulses = 1075.4*(angle/360);
        return pulses;

    public void armRotation(double angle){
        double ArmJointMotorTargetPow =
    }


    }

}

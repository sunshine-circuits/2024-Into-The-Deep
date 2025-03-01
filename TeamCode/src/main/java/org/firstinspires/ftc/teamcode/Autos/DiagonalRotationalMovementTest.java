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

@Autonomous(name="DiagonalRotational")

public class DiagonalRotationalMovementTest extends InchAutoParent {
    @Override
    public void runOpMode() throws InterruptedException
    {
        int DiagonalLevel = 1300;
        int RotationalLevel = 500;
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



        myIMU = hardwareMap.get(IMU.class,"imu");

        waitForStart();
        IMU.Parameters myIMUParameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        myIMU.initialize(myIMUParameters);

        if (opModeIsActive()) {
            myIMU.resetYaw();
            FLMotor.setTargetPosition(DiagonalLevel-RotationalLevel);
            BRMotor.setTargetPosition(DiagonalLevel+RotationalLevel);
            FRMotor.setTargetPosition(RotationalLevel);
            BLMotor.setTargetPosition(-RotationalLevel);
            TimeUnit.SECONDS.sleep(10);

        }
        while (FRMotor.isBusy()||FLMotor.isBusy()||BRMotor.isBusy()||BLMotor.isBusy()) {
            telemetry.addData(" CurrentPosFR",FRMotor.getCurrentPosition());
            telemetry.addData(" CurrentPosFL",FLMotor.getCurrentPosition());
            telemetry.addData(" CurrentPosBR",BRMotor.getCurrentPosition());
            telemetry.addData(" CurrentPosBL",BLMotor.getCurrentPosition());

            telemetry.addData(" TargetPosFR ",FRMotor.getTargetPosition());
            telemetry.addData(" TargetPosFL ",FLMotor.getTargetPosition());
            telemetry.addData(" TargetPosBR ",BRMotor.getTargetPosition());
            telemetry.addData(" TargetPosBL ",BLMotor.getTargetPosition());

            telemetry.addData(" Orientation",myIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
    }

}
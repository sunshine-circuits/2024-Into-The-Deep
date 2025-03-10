package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="FirstOp")
public class ArmTest extends LinearOpMode
{
    // Hardware declarations
    public DcMotor ArmMotor0; //This is the arm on the left(if you are looking at the robot from behind)
    public DcMotor ArmMotor1; //This is the arm on the left(if you are looking at the robot from behind)
    public CRServo Servo0;
    public CRServo Servo1;

    @Override
    public void runOpMode()
    {
        ArmMotor0 = hardwareMap.get(DcMotor.class,"FRMotor");
        ArmMotor1 = hardwareMap.get(DcMotor.class,"FLMotor");
        Servo0 = hardwareMap.get(CRServo.class,"Servo0");
        Servo1 = hardwareMap.get(CRServo.class,"Servo1");

        telemetry.addData("Status","Intialized");
        telemetry.update();


        ArmMotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        double ArmMotor0TargetPow=0;
        double ArmMotor1TargetPow=0;
        double Servo0TargetPow=0;
        double Servo1TargetPow=0;
        while(opModeIsActive())
        {
            telemetry.addData("Status","Running");
            telemetry.update();
            //Motors
            ArmMotor0TargetPow=gamepad1.left_stick_y;
            ArmMotor1TargetPow=gamepad1.right_stick_y;

            DcMotorSimple.Direction LeftServoDirection;
            DcMotorSimple.Direction RightServoDirection;

            //movement for forward&back,      left&right            Rotation


           if(gamepad1.b == true){
               Servo0TargetPow = 1;
               RightServoDirection = DcMotorSimple.Direction.FORWARD;
           }
           else if(gamepad1.a == true) {
               Servo0TargetPow = 1;
               RightServoDirection = DcMotorSimple.Direction.REVERSE;
           }
           else{
               Servo0TargetPow = 0;
               RightServoDirection = DcMotorSimple.Direction.REVERSE;
           }
           if(gamepad1.y == true){
               Servo1TargetPow = 1;
               LeftServoDirection = DcMotorSimple.Direction.FORWARD;
           }
           else if(gamepad1.x == true) {
               Servo1TargetPow = 1;
               LeftServoDirection = DcMotorSimple.Direction.REVERSE;
           }
           else{
               Servo1TargetPow=0;
               LeftServoDirection = DcMotorSimple.Direction.REVERSE;
           }

            ArmMotor0.setPower(ArmMotor0TargetPow);
            ArmMotor1.setPower(ArmMotor1TargetPow);
            Servo0.setPower(Servo1TargetPow);
            Servo0.setDirection(RightServoDirection);
            Servo1.setDirection(LeftServoDirection);
            Servo1.setPower(Servo1TargetPow);
        }
    }

}
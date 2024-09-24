package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@TeleOp(name="FirstOp")
public class FirstOp extends LinearOpMode
{
    // Hardware declarations
    public DcMotor frontRightWheel;
    public DcMotor frontLeftWheel;
    public DcMotor backRightWheel;
    public DcMotor backLeftWheel;
    public DcMotor leftArmMotor; //This is the arm on the left(if you are looking at the robot from behind)
    public double speedMultiplier =1;

    @Override
    public void runOpMode()
    {
        frontRightWheel = hardwareMap.get(DcMotor.class,"FRMotor");
        frontLeftWheel = hardwareMap.get(DcMotor.class,"FLMotor");
        backRightWheel = hardwareMap.get(DcMotor.class,"BRMotor");
        backLeftWheel = hardwareMap.get(DcMotor.class,"BLMotor");
        leftArmMotor = hardwareMap.get(DcMotor.class,"LAMotor");

        telemetry.addData("Status","Intialized");
        telemetry.addData("Handicap", speedMultiplier);
        telemetry.update();

        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        double frontRightTargetPow=0;
        double frontLeftTargetPow=0;
        double backRightTargetPow=0;
        double backLeftTargetPow=0;
        double leftArmTargetPow=0;
        while(opModeIsActive())
        {
            telemetry.addData("Status","Running");
            telemetry.update();
            //Motors

            //movement for forward&back,      left&right            Rotation
            frontRightTargetPow = 0 - gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            frontRightWheel.setPower(speedMultiplier*frontRightTargetPow);
            telemetry.addData("Front Right TP", frontRightTargetPow);
            telemetry.addData("Front Right CP", frontRightWheel.getPower());

            frontLeftTargetPow = 0 + gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
            frontLeftWheel.setPower(speedMultiplier*frontLeftTargetPow);
            telemetry.addData("Front Left TP", frontLeftTargetPow);
            telemetry.addData("Front Left CP", frontLeftWheel.getPower());

            backRightTargetPow = 0 - gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
            backRightWheel.setPower(speedMultiplier*backRightTargetPow);
            telemetry.addData("Back Right TP", backRightTargetPow);
            telemetry.addData("Back Right CP", backRightWheel.getPower());

            backLeftTargetPow = 0 + gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;
            backLeftWheel.setPower(speedMultiplier*backLeftTargetPow);
            telemetry.addData("Back Left TP", backLeftTargetPow);
            telemetry.addData("Back Left CP", backLeftWheel.getPower());



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
            telemetry.addData("Handicap", speedMultiplier);

            if (gamepad1.dpad_up){
                leftArmTargetPow=1;
            }
            else if (gamepad1.dpad_down){
                leftArmTargetPow=-1;
            } else{
                leftArmTargetPow=0;
            }
            telemetry.addData("Handicap", speedMultiplier);

            leftArmMotor.setPower(leftArmTargetPow);
        }
    }

}
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


@TeleOp(name="OutreachOp")
public class OutreachOp extends LinearOpMode
{
    // Hardware declarations
    public DcMotor frontRightWheel;
    public DcMotor frontLeftWheel;
    public DcMotor backRightWheel;
    public DcMotor backLeftWheel;
    public double speedMultiplier =1;

    private void initOpMode() {
        frontRightWheel = hardwareMap.get(DcMotor.class,"FRMotor");
        frontLeftWheel = hardwareMap.get(DcMotor.class,"FLMotor");
        backRightWheel = hardwareMap.get(DcMotor.class,"BRMotor");
        backLeftWheel = hardwareMap.get(DcMotor.class,"BLMotor");

        telemetry.addData("Status","Intialized");
        telemetry.addData("Handicap", speedMultiplier);
        telemetry.update();

        frontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        initOpMode();

        waitForStart();
        double frontRightTargetPow=0;
        double frontLeftTargetPow=0;
        double backRightTargetPow=0;
        double backLeftTargetPow=0;

        while(opModeIsActive())
        {
            telemetry.addData("Status","Running");
            telemetry.update();
            //Motors

            //movement for forward&back,      left&right            Rotation
            frontRightTargetPow = 0 + gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            frontRightWheel.setPower(speedMultiplier*frontRightTargetPow);
            telemetry.addData("Front Right TP", frontRightTargetPow);
            telemetry.addData("Front Right CP", frontRightWheel.getPower());

            frontLeftTargetPow = 0 - gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
            frontLeftWheel.setPower(speedMultiplier*frontLeftTargetPow);
            telemetry.addData("Front Left TP", frontLeftTargetPow);
            telemetry.addData("Front Left CP", frontLeftWheel.getPower());

            backRightTargetPow = 0 + gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
            backRightWheel.setPower(speedMultiplier*backRightTargetPow);
            telemetry.addData("Back Right TP", backRightTargetPow);
            telemetry.addData("Back Right CP", backRightWheel.getPower());

            backLeftTargetPow = 0 - gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
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

            telemetry.update();
        }
    }

}
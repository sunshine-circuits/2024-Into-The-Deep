package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="ViperTest")
public class ViperTest extends LinearOpMode
{
    // Hardware declarations
    public DcMotor ViperMotor;
    public DcMotor ArmJointMotor;
    public boolean isPressed;
    public boolean ArmPos;


    @Override
    public void runOpMode()
    {
        ViperMotor = hardwareMap.get(DcMotor.class,"ViperMotor");
        ArmJointMotor = hardwareMap.get(DcMotor.class, "ArmJointMotor");
        DcMotor.RunMode defaultMode = ArmJointMotor.getMode();

        telemetry.addData("Status","Intialized");
        telemetry.update();
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ViperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
        double ViperMotorTargetPower=0.2;
        double ArmJointTargetPower=0.2;
        while(opModeIsActive())
        {
            telemetry.addData("Status","Running");
            //Motors
            if (gamepad1.dpad_left){
                ViperMotor.setPower(ViperMotorTargetPower);

            }else if (gamepad1.dpad_right){
                ViperMotor.setPower(-ViperMotorTargetPower);
            } else {
                ViperMotor.setPower(0);
            }
            if (gamepad1.left_bumper) {
                if (!isPressed && (ViperMotorTargetPower != 0)) {isPressed = true; ViperMotorTargetPower -= 0.1;}
            } else if (gamepad1.right_bumper) {
                if (!isPressed && (ViperMotorTargetPower != 1)) {isPressed = true; ViperMotorTargetPower += 0.1;}
            } else {isPressed = false;}
            telemetry.addData("Handicap Viper Motor", ViperMotorTargetPower);
            telemetry.update();

            if (gamepad1.dpad_right){
                ArmJointMotor.setMode(defaultMode);
                ArmJointTargetPower=0.2;
                ArmPos=true;
            }
            else if (gamepad1.dpad_left){
                ArmJointMotor.setMode(defaultMode);
                ArmJointTargetPower=-0.2;
                ArmPos=true;
            } else{
                ArmJointTargetPower=1;
                if (ArmPos==true) {
                    ArmJointMotor.setTargetPosition(ArmJointMotor.getCurrentPosition());
                    ArmPos=false;
                }
                ArmJointMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            telemetry.addData("JoinPow", ArmJointTargetPower);

            ArmJointMotor.setPower(ArmJointTargetPower);

        }
    }

}
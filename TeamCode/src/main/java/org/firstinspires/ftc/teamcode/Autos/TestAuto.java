package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TestAuto extends LinearOpMode {

    public DcMotor ExtendArm;

    public void SingleMotorDriver(DcMotor OpMotor, int dist, double pow, String telem){
        int OpPos;
        telemetry.addData("Variables "+telem,"Declared");

        OpPos=OpMotor.getCurrentPosition()+dist;

        telemetry.addData("MotorData "+telem,"Snatched");

        OpMotor.setTargetPosition(OpPos);

        telemetry.addData("TargetPos "+telem,"Declared");
        telemetry.addData("TargetPosFR ",OpPos);

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

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
    }
}

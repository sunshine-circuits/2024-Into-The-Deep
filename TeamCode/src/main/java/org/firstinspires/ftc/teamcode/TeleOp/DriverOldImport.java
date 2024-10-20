package org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.DriverOld;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="DriverDeriver")
public class DriverOldImport extends DriverOld
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initOpMode();
        setupAprilTags();
        setupDrivingVariables();
        waitForStart();

        while(opModeIsActive())
        {
            BasicMotionHandler();
            updateSpeedControl();
            ArmExtendHandler();
            ArmJointHandler();
            ServoHandler();
            AprilTagHandler();
            telemetry.update();
        }
    }

}
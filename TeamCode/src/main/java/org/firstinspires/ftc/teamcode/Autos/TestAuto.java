package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.utils.InchAutoParent;

@Autonomous(name="ThisIsNothingIgnoreIt")
public class TestAuto extends InchAutoParent {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Motors","Grabbing");
        telemetry.update();
        FRMotor = hardwareMap.get(DcMotor.class,"FRMotor");
        FLMotor = hardwareMap.get(DcMotor.class,"FLMotor");
        BRMotor = hardwareMap.get(DcMotor.class,"BRMotor");
        BLMotor = hardwareMap.get(DcMotor.class,"BLMotor");
        ArmJointMotor = hardwareMap.get(DcMotor.class,"ArmJointMotor");
        Headlight = hardwareMap.get(Servo.class, "Headlight");
        ArmHomeSensor = hardwareMap.get(TouchSensor.class, "ArmHomeTouchSensor");
        telemetry.addData("Motors","Got");
        telemetry.update();
        waitForStart();
        DistanceDrive(24,24,0.25,"Frick");

        double autonomouspower = 0.25;//this is the power/speed in which the autonomous runs
        double robotwidth = 18;
        String AutonomousTelemetry = "Hello";//this is the specific telemetry which the drive functions(in the autonomous) use
        DistanceDrive(24, 52, autonomouspower, AutonomousTelemetry);
        DistanceDrive(30, 0, autonomouspower, AutonomousTelemetry);
        DistanceDrive(6, 44 - robotwidth, autonomouspower, AutonomousTelemetry);
        DistanceDrive(-48, 0, autonomouspower, AutonomousTelemetry);
    }
}

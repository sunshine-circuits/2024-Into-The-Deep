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
        double rw = 18;//robot width
        double rl = 18;//robot length
        String AutonomousTelemetry = "Hello";//this is the specific telemetry which the drive functions(in the autonomous) use

        DistanceDrive(45 - rl, 60 - rw, autonomouspower, AutonomousTelemetry);
        DistanceDrive(0, 21, autonomouspower, AutonomousTelemetry);
        DistanceDrive(25,0, autonomouspower, AutonomousTelemetry);
        DistanceDrive(0 , 3 + (rw / 2), autonomouspower, AutonomousTelemetry);


        /*
        Assume the robot dimensions are rw(robot width) x rl(robot length)
        * Starting position(top right wheel): (0 + rl, 36 + rw)
        top right wheel has to go to (45, 96) => drive(45 - (0 + rl), 96 - (36 + rw)) = drive(45 - rl, 60 - rw)
        top right wheel goes to (45, 117) => drive(0, 21)
        top right wheel goes to (69, 117) => drive(25, 0)
        top right wheel goes to (69, 120 + (rw / 2)) => drive(0 , 3 + (rw / 2))

        1 rotation = ? degrees, lets try 90...
        robot rotates 45 degrees counter-clockwise => driver(FR, BR = 0.5, FL, BL = -0.5)
        DistanceDrive(sqrt(15), sqrt(15))
        DistanceDrive(0, 15 * sqrt(2))


        */
    }
}

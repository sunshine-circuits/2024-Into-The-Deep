package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Driver {

    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor rearRightMotor;
    private DcMotor rearLeftMotor;

    private double speedMultiplier;

    public Driver(RobotConfig.Config config) {
        this.frontRightMotor = (DcMotor)config.get("FRMotor");
        this.rearRightMotor = (DcMotor)config.get("BRMotor");
        this.frontLeftMotor = (DcMotor)config.get("FLMotor");
        this.rearLeftMotor = (DcMotor)config.get("BLMotor");
        speedMultiplier = 1.0;

        this.frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rearRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double setSpeedMultiplier(double newSpeed) {
        speedMultiplier = Math.max(0, Math.min(1, newSpeed));
        return speedMultiplier;
    }

    public double updateSpeedMultiplier(double increase) {
        speedMultiplier+=increase;
        speedMultiplier = Math.max(0.0, Math.min(1.0, speedMultiplier));
        return speedMultiplier;
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public void drive(double x, double y, double rotation) {
        y = -y; // Remember, Y stick value is reversed
        x*=1.1; // Counteract imperfect strafing

        double magnitude = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotation), 1);
        double frontLeftPower = (y + x - rotation) / magnitude;
        double rearLeftPower = (y - x - rotation) / magnitude;
        double frontRightPower = (y - x + rotation) / magnitude;
        double rearRightPower = (y + x + rotation) / magnitude;

        frontLeftMotor.setPower(frontLeftPower * speedMultiplier);
        rearLeftMotor.setPower(rearLeftPower * speedMultiplier);
        frontRightMotor.setPower(frontRightPower * speedMultiplier);
        rearRightMotor.setPower(rearRightPower * speedMultiplier);
    }
}





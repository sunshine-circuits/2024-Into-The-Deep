package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class RobotArm {
    private DcMotor armJoint;
    private DcMotor armExtend;

    private CRServo rightClaw;
    private CRServo leftClaw;

    public enum Direction {
        DEPLOY,
        RETRACT,
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }

    public RobotArm(RobotConfig.Config config) {
        this.armJoint = (DcMotor)config.get("ArmJointMotor");
        this.armExtend = (DcMotor)config.get("ArmExtendMotor");
        this.rightClaw = (CRServo)config.get("RightServo");
        this.leftClaw = (CRServo)config.get("LeftServo");

        this.rightClaw.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftClaw.setDirection(DcMotorSimple.Direction.REVERSE);

        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //This method accepts a double of degrees and will rotate to that value as an absolute position.
    //0 degrees is straight up and down. Counter clockwise is negative and clockwise is positive.
    //Ensure the arm will not attempt to travel beyond the acceptable range or collide with the
    //robot.
    public void rotateArmTo(double degrees) {

    }

    //This method accepts a direction and speed and will rotate to a max/min value.
    //0 degrees is straight up and down. Counter clockwise is negative and clockwise is positive.
    //Ensure the arm will not attempt to travel beyond the acceptable range or collide with the
    //robot.
    public void rotateArmManual(Direction direction, double power) {
        switch (direction) {
            case COUNTER_CLOCKWISE: armJoint.setDirection(DcMotorSimple.Direction.FORWARD); armJoint.setPower(power); break;
            case CLOCKWISE: armJoint.setDirection(DcMotorSimple.Direction.REVERSE); armJoint.setPower(power); break;
        }
    }

    //This method accepts a double of inches and will extend/retract to a value as an absolute position.
    //0 inches is fully retracted. X inches is fully extended. Ensure the arm will not attempt to
    //travel beyond the length of the arm nor retract past 0. This should also ensure we can not
    //extend the arm and collide with the robot.
    public void extendArmTo(double inches) {

    }

    //This method accepts a double of power and direction and will extend/retract to a max/min value.
    //0 inches is fully retracted. X inches is fully extended. Ensure the arm will not attempt to
    //travel beyond the length of the arm nor retract past 0. This should also ensure we can not
    //extend the arm and collide with the robot.
    public void extendArmManual(Direction direction, double power) {
        switch (direction) {
            case DEPLOY: armExtend.setDirection(DcMotorSimple.Direction.FORWARD); armExtend.setPower(power); break;
            case RETRACT: armExtend.setDirection(DcMotorSimple.Direction.REVERSE); armExtend.setPower(power); break;
        }
    }

    //This method accepts a double of degrees and will rotate to that value as an absolute position.
    //0 degrees is straight up and down. Counter clockwise is negative and clockwise is positive.
    //Ensure the arm will not attempt to travel beyond the acceptable range or collide with the
    //robot.
    private void setClawPosition(double position, CRServo claw) {
        claw.setPower(position);
        //claw.setPosition(position);
        //claw.getPosition();
    }


    public void setRightClawPosition(double position) {
        setClawPosition(position, this.rightClaw);
    }

    public void setLeftClawPosition(double position) {
        setClawPosition(position, this.leftClaw);
    }
}

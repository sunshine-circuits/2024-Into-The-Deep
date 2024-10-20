package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotArm {
    private DcMotor armJoint;
    private DcMotor armExtend;

    private CRServo rightClaw;
    private CRServo leftClaw;

    public RobotArm(RobotConfig.Config config) {
        this.armJoint = (DcMotor)config.get("ArmJointMotor");
        this.armExtend = (DcMotor)config.get("ArmExtendMotor");
        this.rightClaw = (CRServo)config.get("RightServo");
        this.leftClaw = (CRServo)config.get("LeftServo");
    }

    //This method accepts a double of degrees and will rotate to that value as an absolute position.
    //0 degrees is straight up and down. Counter clockwise is negative and clockwise is positive.
    //Ensure the arm will not attempt to travel beyond the acceptable range or collide with the
    //robot.
    public void rotateArmTo(double degrees) {

    }

    //This method accepts a double of inches and will extend/retract to a value as an absolute position.
    //0 inches is fully retracted. X inches is fully extended. Ensure the arm will not attempt to
    //travel beyond the length of the arm nor retract past 0. This should also ensure we can not
    //extend the arm and collide with the robot.
    public void extendArmTo(double inches) {

    }

    //This method accepts a double of degrees and will rotate to that value as an absolute position.
    //0 degrees is straight up and down. Counter clockwise is negative and clockwise is positive.
    //Ensure the arm will not attempt to travel beyond the acceptable range or collide with the
    //robot.
    private void setClawPosition(double degree, CRServo claw) {

    }


    public void setRightClawPosition(double degree) {
        setClawPosition(degree, this.rightClaw);
    }

    public void setLeftClawPosition(double degree) {
        setClawPosition(degree, this.leftClaw);
    }
}

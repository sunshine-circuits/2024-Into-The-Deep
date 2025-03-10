package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class RobotArm {
    private DcMotor armJoint;
    private DcMotor armExtend;
    private CRServo rightClaw;
    private CRServo leftClaw;
    //This Double tracks the encoder position of the motor in the robot's linear actuator.
    double armExtendPositionRelativeToBasisInPulses;
    //this boolean tracks whether or not armExtend is in RUN_TO_POSITION mode.
    boolean armExtendInRunToPosMode=false;
    //this boolean tracks wheter or not armJoint's position needs set for the braking system.
    boolean armJointPositionNeedsSet=true;
    //this RunMode is the runmode motors are set to when they are first initalized.
    private RunMode defaultMotorMode;
    boolean zeroPositionFound=false;
    double BasisInPulses;

    public enum Direction {
        DEPLOY,
        RETRACT,
        CLOCKWISE,
        COUNTER_CLOCKWISE,
        BRAKE
    }

    public RobotArm(RobotConfig.Config config) {
        this.armJoint = (DcMotor)config.get("ArmJointMotor");
        this.armExtend = (DcMotor)config.get("ArmExtendMotor");
        this.rightClaw = (CRServo)config.get("RightServo");
        this.leftClaw = (CRServo)config.get("LeftServo");
        defaultMotorMode=armExtend.getMode();

        this.rightClaw.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftClaw.setDirection(DcMotorSimple.Direction.REVERSE);

        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //this double contains the number of pulses the encoder runs to move one rotation
    private final double PULSES_PER_ROTATION = 537.7;

    //this method accepts a double of inches and will return the number of pulses armExtend needs to rotate to move that far.
//    public double InchesToPulses(double inches){
//        return (inches/0.5)*PULSES_PER_ROTATION;
//    }

    //this method is a void that swaps armExtend to DefaultMotorMode if it is busy.
//    public void runBackgroundArmExtendProcesses(){
//        //swaps armExtend to DefaultMotorMode if it is busy
//        if(armExtend.isBusy()){
//            armExtendInRunToPosMode=true;
//        }else{
//            if(armExtendInRunToPosMode){
//                armExtend.setMode(defaultMotorMode);
//            }
//        }
//        if(zeroPositionFound==false){
//            BasisInPulses=armExtend.getCurrentPosition();
//        }
//        armExtendPositionRelativeToBasisInPulses=armExtend.getCurrentPosition()-BasisInPulses;
//    }


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
            case COUNTER_CLOCKWISE: armJoint.setMode(defaultMotorMode); armJoint.setDirection(DcMotorSimple.Direction.FORWARD); armJoint.setPower(power); armJointPositionNeedsSet=true; break;
            case CLOCKWISE: armJoint.setMode(defaultMotorMode); armJoint.setDirection(DcMotorSimple.Direction.REVERSE); armJoint.setPower(power); armJointPositionNeedsSet=true; break;
            case BRAKE:
                armJoint.setDirection(DcMotorSimple.Direction.FORWARD);
                armJoint.setPower(power);
                if (armJointPositionNeedsSet) {

                    armJoint.setTargetPosition(armJoint.getCurrentPosition());
                    armJointPositionNeedsSet=false;
                }
                armJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION); break;
        }
    }

    //This method accepts a double of inches and will extend/retract to a value as an absolute position.
    //0 inches is fully retracted. X inches is fully extended. Ensure the arm will not attempt to
    //travel beyond the length of the arm nor retract past 0. This should also ensure we can not
    //extend the arm and collide with the robot.
    public void extendArmTo(double inches){
        double pulsesFromZero=(inches/0.5)*PULSES_PER_ROTATION;
        double pulsesNeeded=pulsesFromZero-armExtendPositionRelativeToBasisInPulses;
        armExtendPositionRelativeToBasisInPulses=armExtendPositionRelativeToBasisInPulses+pulsesNeeded;
        armExtend.setTargetPosition((int) (armExtend.getCurrentPosition()+pulsesNeeded));
        armExtend.setMode(RunMode.RUN_TO_POSITION);
    }

    //This method accepts a double of power and direction and will extend/retract to a max/min value.
    //0 inches is fully retracted. X inches is fully extended. Ensure the arm will not attempt to
    //travel beyond the length of the arm nor retract past 0. This should also ensure we can not
    //extend the arm and collide with the robot.
    public void extendArmManual(Direction direction, double power) {
        switch (direction) {
            case DEPLOY:
//            if(armExtendPositionRelativeToBasisInPulses>=InchesToPulses(7.5)){
//
//            }else{
//                armExtend.setDirection(DcMotorSimple.Direction.FORWARD);
//                armExtend.setPower(power);
//            }
//            break;
                armExtend.setDirection(DcMotorSimple.Direction.FORWARD); armExtend.setPower(power); break;
            case RETRACT: armExtend.setDirection(DcMotorSimple.Direction.REVERSE); armExtend.setPower(power); break;
        }
    }
    //this method, when called, reverses the motion of the arm
//    public void resetExtender(){
//        armExtend.setTargetPosition((int) (armExtend.getCurrentPosition()- InchesToPulses(8)));
//        armExtend.setMode(RunMode.RUN_TO_POSITION);
//        armExtendPositionRelativeToBasisInPulses=0;
//    }

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

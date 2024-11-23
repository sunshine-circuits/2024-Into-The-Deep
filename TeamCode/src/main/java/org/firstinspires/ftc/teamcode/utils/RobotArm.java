package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotArm {
    public DcMotor armJoint;
    private DcMotor armExtend;
    private Servo rightClaw;
    private Servo leftClaw;
    private Servo headlight;
    public TouchSensor JointTouchSensor;
    public RevColorSensorV3 clawDistanceSensor;
    //This Double tracks the encoder position of the motor in the robot's linear actuator.
    double armExtendPositionRelativeToBasisInPulses;
    //this boolean tracks whether or not armExtend is in RUN_TO_POSITION mode.
    public boolean jointLimitSet=false;
    //this boolean tracks whether or not armJoint's position needs set for the braking system.
    boolean armJointPositionNeedsSet=true;
    //this RunMode is the runmode motors are set to when they are first initialized.
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
        this.rightClaw = (Servo)config.get("RightServo");
        this.leftClaw = (Servo)config.get("LeftServo");
        this.headlight = (Servo)config.get("Headlight");
        this.JointTouchSensor = (TouchSensor)config.get("TouchSensor");
        this.clawDistanceSensor = (RevColorSensorV3)config.get("DistanceSensor");

        this.headlight.setPosition(1);
        defaultMotorMode=armExtend.getMode();

        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtend.setMode(RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(defaultMotorMode);
        armJoint.setMode(RunMode.STOP_AND_RESET_ENCODER);
        armJoint.setMode(defaultMotorMode);
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

    public void moveToOrigin() throws Exception {
        armJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armJoint.setTargetPosition(-2000);
        armJoint.setPower(0.2);
        while(armJoint.isBusy() && !JointTouchSensor.isPressed()) {

        }
        //Same thing, but by increments.
        //        int count = 0;
        //        while (!JointTouchSensor.isPressed()){
        //            armJoint.setTargetPosition(-10 * count);
        //            armJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //            armJoint.setPower(0.2);
        //            while(armJoint.isBusy());
        //            count++;
        //        }
        if (!JointTouchSensor.isPressed()) {
            throw new Exception("Could not move to origin");
        }
        armJoint.setMode(RunMode.STOP_AND_RESET_ENCODER);
        armJoint.setMode(defaultMotorMode);


    }

    //This method accepts a direction and speed and will rotate to a max/min value.
    //0 degrees is straight up and down. Counter clockwise is negative and clockwise is positive.
    //Ensure the arm will not attempt to travel beyond the acceptable range or collide with the
    //robot.
    public void rotateArmManual(Direction direction, double power) {
        if((armJoint.getCurrentPosition() < 830)&&(clawDistanceSensor.getDistance(DistanceUnit.CM)>2)) {
            armJoint.setDirection(DcMotorSimple.Direction.FORWARD);
            armJoint.setTargetPosition(845);
            armJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }else {
            switch (direction) {
                case COUNTER_CLOCKWISE:
                    armJoint.setMode(defaultMotorMode);
                    armJoint.setDirection(DcMotorSimple.Direction.FORWARD);
                    armJoint.setPower(power);
                    armJointPositionNeedsSet = true;
                    break;
                case CLOCKWISE:
                    if ((armJoint.getCurrentPosition()>830)||(clawDistanceSensor.getDistance(DistanceUnit.CM)<=2)) {
                        armJoint.setMode(defaultMotorMode);
                        armJoint.setDirection(DcMotorSimple.Direction.REVERSE);
                        armJoint.setPower(power);
                        armJointPositionNeedsSet = true;
                    }
                    break;
                case BRAKE:
                    armJoint.setDirection(DcMotorSimple.Direction.FORWARD);
                    armJoint.setPower(power);
                    if (armJointPositionNeedsSet) {

                        armJoint.setTargetPosition(armJoint.getCurrentPosition());
                        armJointPositionNeedsSet = false;
                    }
                    armJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    break;
            }
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
                armExtend.setDirection(DcMotorSimple.Direction.FORWARD);
                armExtend.setPower(power);
                break;
            case RETRACT:
                armExtend.setDirection(DcMotorSimple.Direction.REVERSE);
                armExtend.setPower(power);
                break;
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
    private void setClawPosition(double position, Servo claw) {
        claw.setPosition(position);
        //claw.getPosition();
    }


    public void setRightClawPosition(double position) {
        setClawPosition(position, this.rightClaw);
    }

    public void setLeftClawPosition(double position) {
        setClawPosition(position, this.leftClaw);
    }

    //using Headlight for Telemetry
    public void SetLightToDistance(){
        headlight.setPosition(clawDistanceSensor.getDistance(DistanceUnit.CM)/6);
    }
}

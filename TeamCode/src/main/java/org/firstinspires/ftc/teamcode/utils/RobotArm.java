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
    public DcMotor armExtend;
    public DcMotor hangArm;
    public Servo rightClaw;
    public Servo leftClaw;
    public Servo headlight;
    public TouchSensor JointTouchSensor;
    //This Double tracks the encoder position of the motor in the robot's linear actuator.
    double armExtendPositionRelativeToBasisInPulses;
    //this boolean tracks whether or not armExtend is in RUN_TO_POSITION mode.
    public boolean jointLimitSet=false;
    //this boolean tracks whether or not armJoint's position needs set for the braking system.
    boolean armJointPositionNeedsSet=true;
    boolean armExtendPositionNeedsSet = true;
    boolean hangArmPositionNeedsSet=true;
    //this RunMode is the runmode motors are set to when they are first initialized.
    private RunMode defaultMotorMode;
    public double PowerDecrease = 0.5;

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
        this.hangArm = (DcMotor)config.get("hangArm");
        this.rightClaw = (Servo)config.get("RightServo");
        this.leftClaw = (Servo)config.get("LeftServo");
        this.headlight = (Servo)config.get("Headlight");
        this.JointTouchSensor = (TouchSensor)config.get("TouchSensor");

        this.headlight.scaleRange(0,1);
        this.headlight.setPosition(0.001);
        defaultMotorMode=armExtend.getMode();

        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armJoint.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtend.setMode(RunMode.STOP_AND_RESET_ENCODER);
        armExtend.setMode(defaultMotorMode);
        armJoint.setMode(RunMode.STOP_AND_RESET_ENCODER);
        armJoint.setMode(defaultMotorMode);
        hangArm.setMode(RunMode.STOP_AND_RESET_ENCODER);
        hangArm.setMode(defaultMotorMode);
    }

    //this double contains the number of pulses the encoder runs to move one rotation
    private final double PULSES_PER_ROTATION = 537.7;

    public void moveToOrigin() throws Exception {
        armJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armJoint.setTargetPosition(-2000);
        armJoint.setPower(0.2);
        while(armJoint.isBusy() && !JointTouchSensor.isPressed()) {

        }
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

       if((Math.abs(armExtend.getCurrentPosition())>= 375)&&(armJoint.getCurrentPosition() >= 800)){
           armJoint.setDirection(DcMotorSimple.Direction.FORWARD);
           armJoint.setTargetPosition(785);
           armJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           armJoint.setPower(1);
           armJointPositionNeedsSet = true;

       }else if((armJoint.getCurrentPosition() < 635)&&(Math.abs(armExtend.getCurrentPosition())>=550)) {
            armJoint.setDirection(DcMotorSimple.Direction.FORWARD);
            armJoint.setTargetPosition(650);
            armJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armJoint.setPower(1);
           armJointPositionNeedsSet = true;
        }else {
            switch (direction) {
                case COUNTER_CLOCKWISE:
                    if ((armJoint.getCurrentPosition() < 770) || (Math.abs(armExtend.getCurrentPosition()) <= 375)) {
                        armJoint.setMode(defaultMotorMode);
                        armJoint.setDirection(DcMotorSimple.Direction.FORWARD);
                        if (armJoint.getCurrentPosition() > 800) {
                            armJoint.setPower(power/3);
                        } else {
                            armJoint.setPower(power);
                        }
                    }
                    armJointPositionNeedsSet = true;
                    break;
                case CLOCKWISE:
                    if ((armJoint.getCurrentPosition() > 635) || (Math.abs(armExtend.getCurrentPosition()) <= 550)) {
                        armJoint.setMode(defaultMotorMode);
                        armJoint.setDirection(DcMotorSimple.Direction.REVERSE);
                        if (armJoint.getCurrentPosition() < 635) {
                            armJoint.setPower(power * PowerDecrease);
                        } else {
                            armJoint.setPower(power);
                        }
                    }
                    armJointPositionNeedsSet = true;
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

    //This method accepts a double that represents power and a Direction that represents it's current directio
    public void extendArmManual(Direction direction, double power) {
        if(Math.abs(armExtend.getCurrentPosition())>=3075){
            armExtend.setDirection(DcMotorSimple.Direction.FORWARD);
            armExtend.setTargetPosition((Math.abs(armExtend.getCurrentPosition())/armExtend.getCurrentPosition())*3025);
            armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armExtend.setPower(power);
        } else /*
        if((armJoint.getCurrentPosition() > 800)&&(Math.abs(armExtend.getCurrentPosition())>=375)) {
            armExtend.setDirection(DcMotorSimple.Direction.FORWARD);
            armExtend.setTargetPosition((Math.abs(armExtend.getCurrentPosition())/armExtend.getCurrentPosition())*365);
            armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armExtend.setPower(1);
        } else //*/{
            switch (direction) {
                case DEPLOY:
//            if(armExtendPositionRelativeToBasisInPulses>=InchesToPulses(7.5)){
//
//            }else{
//                armExtend.setDirection(DcMotorSimple.Direction.FORWARD);
//                armExtend.setPower(power);
//            }
//            break;
                    armExtend.setMode(defaultMotorMode);
                    armExtend.setDirection(DcMotorSimple.Direction.FORWARD);
                    armExtend.setPower(power);
                    armExtendPositionNeedsSet = true;
                    break;
                case RETRACT:
                    if(Math.abs(armExtend.getCurrentPosition())<=3000) {
                        if ((((armJoint.getCurrentPosition() < 635) || (armJoint.getCurrentPosition() > 800))&&(Math.abs(armExtend.getCurrentPosition()) <= 350))||((armJoint.getCurrentPosition() > 635)&&(armJoint.getCurrentPosition() < 800))){
                            armExtend.setMode(defaultMotorMode);
                            armExtend.setDirection(DcMotorSimple.Direction.REVERSE);
                            armExtend.setPower(power);

                        }
                    }
                    armExtendPositionNeedsSet = true;
                    break;
                case BRAKE:
                    if(Math.abs(armExtend.getCurrentPosition())>=250) {
                        armExtend.setDirection(DcMotorSimple.Direction.FORWARD);
                        armExtend.setPower(power);
                        if (armExtendPositionNeedsSet) {

                            armExtend.setTargetPosition(armExtend.getCurrentPosition());
                            armExtendPositionNeedsSet = false;
                        }
                        armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    break;
            }
        }
    }

    //This method accepts a direction and speed and will rotate the hang arm based on power level

    //I have no idea why this doesn't do what I thought it would.
    //it's just supposed to rotate when power is applied then brake when it's not.
    //instead, it just keeps rotating and doesn't stop.
    //to fix this, the BRAKE case simply Brakes instead of actively trying to keep itself in it's current position.
    public void rotateHangArm(Direction direction, double power) {
        switch (direction) {
            case COUNTER_CLOCKWISE:
                hangArm.setMode(defaultMotorMode);
                hangArm.setPower(power);
                hangArmPositionNeedsSet = true;
                break;
            case CLOCKWISE:
                hangArm.setMode(defaultMotorMode);
                hangArm.setPower(-power);
                hangArmPositionNeedsSet = true;
                break;
            case BRAKE:
                hangArm.setPower(0);
                //if (hangArmPositionNeedsSet) {
                //    hangArm.setTargetPosition(hangArm.getCurrentPosition());
                //    hangArmPositionNeedsSet = false;
                //}
                //hangArm.setMode(RunMode.RUN_TO_POSITION);
                break;
        }
    }

    //this method accepts a double from 0 to 1 and a Servo. it then rotates to that double's 'position' on the Servo
    //the 'position' of a double the Servo can be understood as the fraction of the servo's maximum clockwise rotation that it's current clockwise rotation represents, where 0 is it's maximum counterclockwise rotation and 1 is it's maximum clockwise rotation
    private void setClawPosition(double position, Servo claw) {
        claw.setPosition(position);
    }

    public void setRightClawPosition(double position) {
        setClawPosition(position, this.rightClaw);
    }

    public void setLeftClawPosition(double position) {
        setClawPosition(position, this.leftClaw);
    }

}

package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.utils.Keybind;
import org.firstinspires.ftc.teamcode.utils.RobotArm;
import org.firstinspires.ftc.teamcode.utils.RobotConfig;
import org.firstinspires.ftc.teamcode.utils.*;

@TeleOp(name="ThisIsTheCorrectTeleOp")
public class ExampleTeleOp extends OpMode {
    private Driver driver;
    private RobotArm arm;
    private Keybind keybind;
    private boolean speedDebounce = false;
    private final double ARM_SPEED_LIMIT = 0.20;
    public boolean aprilTagDetected;
    public boolean ArmParallel;
    public boolean ArmEncoderResetCheck=false;
    public boolean ExtendEncoderResetCheck=false;
    public boolean upMacroOn=false;


    @Override
    public void init() {
        RobotConfig.Config.LM0.initConfig(hardwareMap);
        driver = new Driver(RobotConfig.Config.LM0);
        arm = new RobotArm(RobotConfig.Config.LM0);
        keybind = new Keybind(gamepad1, gamepad2);

        keybind.addOrUpdate("drive_y", Keybind.Input.GAMEPAD_1_LEFT_STICK_Y);
        keybind.addOrUpdate("drive_x", Keybind.Input.GAMEPAD_1_LEFT_STICK_X);
        keybind.addOrUpdate("rotate", Keybind.Input.GAMEPAD_1_RIGHT_STICK_X);
        keybind.addOrUpdate("claw_open", Keybind.Input.GAMEPAD_2_RIGHT_BUMPER);
        keybind.addOrUpdate("claw_close", Keybind.Input.GAMEPAD_2_LEFT_BUMPER);
        keybind.addOrUpdate("arm_rotation_ccw", Keybind.Input.GAMEPAD_2_LEFT_TRIGGER);
        keybind.addOrUpdate("arm_rotation_cw", Keybind.Input.GAMEPAD_2_RIGHT_TRIGGER);
        keybind.addOrUpdate("arm_extend", Keybind.Input.GAMEPAD_2_A);
        keybind.addOrUpdate("arm_retract", Keybind.Input.GAMEPAD_2_B);
        keybind.addOrUpdate("upmacro", Keybind.Input.GAMEPAD_2_Y);
        keybind.addOrUpdate("control_left_claw", Keybind.Input.GAMEPAD_2_LEFT_STICK_X);
        keybind.addOrUpdate("control_right_claw", Keybind.Input.GAMEPAD_2_RIGHT_STICK_X);
        keybind.addOrUpdate("increase_speed", Keybind.Input.GAMEPAD_1_RIGHT_BUMPER);
        keybind.addOrUpdate("decrease_speed", Keybind.Input.GAMEPAD_1_LEFT_BUMPER);
        keybind.addOrUpdate("Reset",Keybind.Input.GAMEPAD_2_X);

        try {
            arm.moveToOrigin();
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
//        arm.resetExtender();
    }

    @Override
    public void loop() {

        arm.headlight.scaleRange(0,1);
        arm.headlight.setPosition(0.001);

        if(keybind.poll("upmacro")){
            arm.ArmUpMacro();
            upMacroOn=true;
        }
        telemetry.addData("Up Macro Active",upMacroOn);

        driver.drive(keybind.pollValue("drive_x"), keybind.pollValue("drive_y"), keybind.pollValue("rotate"));

        telemetry.addData("Current Speed: ", driver.getSpeedMultiplier());

        telemetry.addData("-------------------------","");
        telemetry.addData("Arm Extend Target Position",arm.armExtend.getTargetPosition());
        telemetry.addData("Arm Extend Current Position",arm.armExtend.getCurrentPosition());
        if (keybind.poll("arm_extend")) {
            arm.extendArmManual(RobotArm.Direction.DEPLOY, 0.7);
            upMacroOn=false;
        } else if (keybind.poll("arm_retract")) {
            arm.extendArmManual(RobotArm.Direction.RETRACT, 0.7);
            upMacroOn=false;
        } else {
            if(upMacroOn==false) {
                arm.extendArmManual(RobotArm.Direction.BRAKE, 1);
            }
        }
        telemetry.addData("--------------------------","");
        telemetry.addData("Arm Limit Set?",arm.jointLimitSet);
        telemetry.addData("Arm touch sensor pressed",arm.JointTouchSensor.isPressed());
        telemetry.addData("---------------------------","");
        telemetry.addData("Arm Joint Target Position",arm.armJoint.getTargetPosition());
        telemetry.addData("Arm Joint Current Position",arm.armJoint.getCurrentPosition());

        //makes arm rotate away from robot
        if (keybind.pollValue("arm_rotation_ccw") > 0) {
            upMacroOn=false;
            if(arm.armJoint.getCurrentPosition() > 1100){
                arm.rotateArmManual(RobotArm.Direction.COUNTER_CLOCKWISE, Math.max(keybind.pollValue("arm_rotation_ccw"), ARM_SPEED_LIMIT*(1/6)));
                if (!ArmParallel) {
                    ArmParallel = true;
                }

            }else if(arm.armJoint.getCurrentPosition() > 800){
                arm.rotateArmManual(RobotArm.Direction.COUNTER_CLOCKWISE, Math.max(keybind.pollValue("arm_rotation_ccw"), ARM_SPEED_LIMIT*(2/6)));
                if (ArmParallel) {
                    ArmParallel = false;
                }
            }else {
                arm.rotateArmManual(RobotArm.Direction.COUNTER_CLOCKWISE, Math.max(keybind.pollValue("arm_rotation_ccw"), ARM_SPEED_LIMIT));
                if (ArmParallel) {
                    ArmParallel = false;
                }

            }
        } else// makes arm rotate towards robot
            if (keybind.pollValue("arm_rotation_cw") > 0) {
                upMacroOn=false;
                if(arm.armJoint.getCurrentPosition() > 800){
                    arm.rotateArmManual(RobotArm.Direction.CLOCKWISE, Math.max(keybind.pollValue("arm_rotation_cw"), ARM_SPEED_LIMIT));
                }else{
                    arm.rotateArmManual(RobotArm.Direction.CLOCKWISE, Math.max(keybind.pollValue("arm_rotation_cw"), ARM_SPEED_LIMIT*(2/6)));
                }
            } else {
                if(upMacroOn==false){
                    arm.rotateArmManual(RobotArm.Direction.BRAKE, 0.5);
                }
        }

        telemetry.addData("----------------------------","");

        telemetry.addData("LeftClawPos", arm.leftClaw.getPosition());
        telemetry.addData("RightClawPos", arm.rightClaw.getPosition());
        if (keybind.poll("claw_open")) {
            arm.setRightClawPosition(0.57);
            arm.setLeftClawPosition(0.43);
        } else if (keybind.poll("claw_close")) {
            arm.setRightClawPosition(0.49);
            arm.setLeftClawPosition(0.51);
        } else {
            arm.setRightClawPosition(0.52);
            arm.setLeftClawPosition(0.48);
        }

        if (keybind.pollValue("control_left_claw") != 0) {
            arm.setLeftClawPosition(keybind.pollValue("control_left_claw"));
        }

        if (keybind.pollValue("control_right_claw") != 0) {
            arm.setRightClawPosition(keybind.pollValue("control_right_claw"));
        }

        if (keybind.poll("increase_speed")) {
            if (!speedDebounce) {
                driver.updateSpeedMultiplier(0.1);
            }
            speedDebounce = true;
        } else if (keybind.poll("decrease_speed")) {
            if (!speedDebounce) {
                driver.updateSpeedMultiplier(-0.1);
            }
            speedDebounce = true;
        } else {
            speedDebounce = false;
        }

        telemetry.addData("Arm Reset?", ArmEncoderResetCheck);
        if ((arm.JointTouchSensor.isPressed())){
            if(ArmEncoderResetCheck =false) {
                arm.armJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.armJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ArmEncoderResetCheck =true;
            }else{
                if((arm.JointTouchSensor.isPressed())==false){
                    ArmEncoderResetCheck =false;
                }
            }
        }else{
            ArmEncoderResetCheck =false;
        }
        telemetry.addData("Extend Reset?", ExtendEncoderResetCheck);
        telemetry.addData("What the duck?", keybind.poll("Reset"));
        if ((keybind.poll("Reset"))){
            if(ExtendEncoderResetCheck==false) {
                arm.armExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.armExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ExtendEncoderResetCheck=true;
            }else{
                if((keybind.poll("Reset"))==false){
                    ExtendEncoderResetCheck=false;
                }
            }
        }else{
            ExtendEncoderResetCheck=false;
        }

    }
}

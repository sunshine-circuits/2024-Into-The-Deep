package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Keybind;
import org.firstinspires.ftc.teamcode.utils.RobotArm;
import org.firstinspires.ftc.teamcode.utils.RobotConfig;
import org.firstinspires.ftc.teamcode.utils.*;

@TeleOp(name="ExampleTeleOp.java")
public class ExampleTeleOp extends OpMode {
    private Driver driver;
    private RobotArm arm;
    private Keybind keybind;
    private boolean speedDebounce = false;
    private final double ARM_SPEED_LIMIT = 0.2;

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
        keybind.addOrUpdate("left_claw_down", Keybind.Input.GAMEPAD_2_DPAD_LEFT);
        keybind.addOrUpdate("left_claw_up", Keybind.Input.GAMEPAD_2_DPAD_RIGHT);
        keybind.addOrUpdate("right_claw_down", Keybind.Input.GAMEPAD_2_DPAD_DOWN);
        keybind.addOrUpdate("right_claw_up", Keybind.Input.GAMEPAD_2_DPAD_UP);
        keybind.addOrUpdate("arm_rotation_ccw", Keybind.Input.GAMEPAD_2_LEFT_TRIGGER);
        keybind.addOrUpdate("arm_rotation_cw", Keybind.Input.GAMEPAD_2_RIGHT_TRIGGER);
        keybind.addOrUpdate("arm_extend", Keybind.Input.GAMEPAD_2_A);
        keybind.addOrUpdate("arm_retract", Keybind.Input.GAMEPAD_2_B);
        keybind.addOrUpdate("control_left_claw", Keybind.Input.GAMEPAD_2_LEFT_STICK_X);
        keybind.addOrUpdate("control_right_claw", Keybind.Input.GAMEPAD_2_RIGHT_STICK_X);
        keybind.addOrUpdate("increase_speed", Keybind.Input.GAMEPAD_1_RIGHT_BUMPER);
        keybind.addOrUpdate("decrease_speed", Keybind.Input.GAMEPAD_1_LEFT_BUMPER);

//        arm.resetExtender();
    }

    @Override
    public void loop() {
        driver.drive(keybind.pollValue("drive_x"), keybind.pollValue("drive_y"), keybind.pollValue("rotate"));

        telemetry.addData("Current Speed: ", driver.getSpeedMultiplier());

        if (keybind.poll("arm_extend")) {
            arm.extendArmManual(RobotArm.Direction.DEPLOY, 1);
        } else if (keybind.poll("arm_retract")) {
            arm.extendArmManual(RobotArm.Direction.RETRACT, 1);
        } else {
            arm.extendArmManual(RobotArm.Direction.RETRACT, 0);
        }
//        arm.runBackgroundArmExtendProcesses();
        telemetry.addData("Arm Limit Set?",arm.jointLimitSet);
        telemetry.addData("Arm touch sensor pressed",arm.JointTouchSensor.isPressed());
        if(arm.jointLimitSet==false){
            arm.rotateArmManual(RobotArm.Direction.CLOCKWISE, ARM_SPEED_LIMIT);
        }else if (keybind.pollValue("arm_rotation_ccw") > 0) {
            arm.rotateArmManual(RobotArm.Direction.COUNTER_CLOCKWISE, Math.max(keybind.pollValue("arm_rotation_ccw"), ARM_SPEED_LIMIT));
        } else if (keybind.pollValue("arm_rotation_cw") > 0) {
            arm.rotateArmManual(RobotArm.Direction.CLOCKWISE, Math.max(keybind.pollValue("arm_rotation_cw"), ARM_SPEED_LIMIT));
        } else {
            arm.rotateArmManual(RobotArm.Direction.BRAKE, 0.3);
        }

        if (keybind.poll("left_claw_down")) {
            arm.setLeftClawPosition(0);
        }else if (keybind.poll("left_claw_up")) {
            arm.setLeftClawPosition(1);
        }else if (keybind.poll("right_claw_down")) {
            arm.setRightClawPosition(0);
        }else if (keybind.poll("right_claw_up")) {
            arm.setRightClawPosition(1);
        }else if (keybind.poll("claw_open")) {
            arm.setRightClawPosition(0.15);
            arm.setLeftClawPosition(0.85);
        } else if (keybind.poll("claw_close")) {
            arm.setRightClawPosition(1);
            arm.setLeftClawPosition(0);
        } else {
            arm.setRightClawPosition(0.45);
            arm.setLeftClawPosition(0.55);
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
    }
}

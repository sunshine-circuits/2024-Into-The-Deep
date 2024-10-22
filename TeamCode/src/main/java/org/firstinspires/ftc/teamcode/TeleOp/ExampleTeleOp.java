package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.utils.Driver;
import org.firstinspires.ftc.teamcode.utils.Keybind;
import org.firstinspires.ftc.teamcode.utils.RobotArm;
import org.firstinspires.ftc.teamcode.utils.RobotConfig;

public class ExampleTeleOp extends OpMode {
private Driver driver;
private RobotArm arm;
private Keybind keybind;
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
        keybind.addOrUpdate("increase_speed", Keybind.Input.GAMEPAD_1_RIGHT_BUMPER);
        keybind.addOrUpdate("decrease_speed", Keybind.Input.GAMEPAD_1_LEFT_BUMPER);
    }

    @Override
    public void loop() {
        driver.drive(keybind.pollValue("drive_x"), keybind.pollValue("drive_y"), keybind.pollValue("rotate"));

        if (keybind.poll("arm_extend")) {
            arm.extendArmManual(RobotArm.Direction.DEPLOY, 1);
        } else if (keybind.poll("arm_retract")) {
            arm.extendArmManual(RobotArm.Direction.RETRACT, 1);
        }

        if (keybind.pollValue("arm_rotation_ccw") > 0) {
            arm.rotateArmManual(RobotArm.Direction.COUNTER_CLOCKWISE, keybind.pollValue("arm_rotation_ccw"));
        } else if (keybind.pollValue("arm_retract") > 0) {
            arm.rotateArmManual(RobotArm.Direction.CLOCKWISE, keybind.pollValue("arm_rotation_ccw"));
        }

        if (keybind.poll("claw_open")) {
            arm.setRightClawPosition(0);
            arm.setLeftClawPosition(0);
        } else if (keybind.poll("claw_close")) {
            arm.setRightClawPosition(1);
            arm.setLeftClawPosition(1);
        }

        if (keybind.poll("increase_speed")) {
            driver.updateSpeedMultiplier(.1);
        } else if (keybind.poll("decrease_speed")) {
            driver.updateSpeedMultiplier(-.1);
        }
    }
}

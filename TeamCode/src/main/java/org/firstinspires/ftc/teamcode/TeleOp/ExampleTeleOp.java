package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    private final double ARM_SPEED_LIMIT = 0.3;
    public boolean aprilTagDetected;
    public boolean Hanging=false;
    public boolean HangRight=false;

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
        keybind.addOrUpdate("disengage_hang", Keybind.Input.GAMEPAD_2_DPAD_DOWN);
        keybind.addOrUpdate("engage_hang", Keybind.Input.GAMEPAD_2_DPAD_UP);
        keybind.addOrUpdate("hang_change_direction_right", Keybind.Input.GAMEPAD_2_DPAD_RIGHT);
        keybind.addOrUpdate("hang_change_direction_left", Keybind.Input.GAMEPAD_2_DPAD_LEFT);
        keybind.addOrUpdate("arm_rotation_ccw", Keybind.Input.GAMEPAD_2_LEFT_TRIGGER);
        keybind.addOrUpdate("arm_rotation_cw", Keybind.Input.GAMEPAD_2_RIGHT_TRIGGER);
        keybind.addOrUpdate("arm_extend", Keybind.Input.GAMEPAD_2_A);
        keybind.addOrUpdate("arm_retract", Keybind.Input.GAMEPAD_2_B);
        keybind.addOrUpdate("hang_rotation_ccw", Keybind.Input.GAMEPAD_2_X);
        keybind.addOrUpdate("hang_rotation_cw", Keybind.Input.GAMEPAD_2_Y);
        keybind.addOrUpdate("control_left_claw", Keybind.Input.GAMEPAD_2_LEFT_STICK_X);
        keybind.addOrUpdate("control_right_claw", Keybind.Input.GAMEPAD_2_RIGHT_STICK_X);
        keybind.addOrUpdate("increase_speed", Keybind.Input.GAMEPAD_1_RIGHT_BUMPER);
        keybind.addOrUpdate("decrease_speed", Keybind.Input.GAMEPAD_1_LEFT_BUMPER);

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

        driver.drive(keybind.pollValue("drive_x"), keybind.pollValue("drive_y"), keybind.pollValue("rotate"));

        telemetry.addData("Current Speed: ", driver.getSpeedMultiplier());

        telemetry.addData("-------------------------","");
        telemetry.addData("Arm Extend Target Position",arm.armExtend.getTargetPosition());
        telemetry.addData("Arm Extend Current Position",arm.armExtend.getCurrentPosition());
        if (keybind.poll("arm_extend")) {
            arm.extendArmManual(RobotArm.Direction.DEPLOY, 0.5);
        } else if (keybind.poll("arm_retract")) {
            arm.extendArmManual(RobotArm.Direction.RETRACT, 0.5);
        } else {
            arm.extendArmManual(RobotArm.Direction.BRAKE, 1);
        }
        telemetry.addData("--------------------------","");
        telemetry.addData("Arm Limit Set?",arm.jointLimitSet);
        telemetry.addData("Arm touch sensor pressed",arm.JointTouchSensor.isPressed());
        telemetry.addData("---------------------------","");
        telemetry.addData("Arm Joint Target Position",arm.armJoint.getTargetPosition());
        telemetry.addData("Arm Joint Current Position",arm.armJoint.getCurrentPosition());

        //makes arm rotate away from robot
        if (keybind.pollValue("arm_rotation_ccw") > 0) {
            if(arm.armJoint.getCurrentPosition() > 800){
                arm.rotateArmManual(RobotArm.Direction.COUNTER_CLOCKWISE, Math.max(keybind.pollValue("arm_rotation_ccw"), ARM_SPEED_LIMIT*(2/4)));
            }else{
                arm.rotateArmManual(RobotArm.Direction.COUNTER_CLOCKWISE, Math.max(keybind.pollValue("arm_rotation_ccw"), ARM_SPEED_LIMIT*(4/2)));
            }
        } else// makes arm rotate towards robot
            if (keybind.pollValue("arm_rotation_cw") > 0) {
                if(arm.armJoint.getCurrentPosition() > 800){
                    arm.rotateArmManual(RobotArm.Direction.CLOCKWISE, Math.max(keybind.pollValue("arm_rotation_cw"), ARM_SPEED_LIMIT*(4/2)));
                }else{
                    arm.rotateArmManual(RobotArm.Direction.CLOCKWISE, Math.max(keybind.pollValue("arm_rotation_cw"), ARM_SPEED_LIMIT*(2/4)));
                }
               // arm.rotateArmManual(RobotArm.Direction.CLOCKWISE, Math.max(keybind.pollValue("arm_rotation_cw"), ARM_SPEED_LIMIT));
        } else {
            arm.rotateArmManual(RobotArm.Direction.BRAKE, 0.5);
        }

        telemetry.addData("----------------------------","");
        telemetry.addData("Hang Arm Target Position",arm.hangArm.getTargetPosition());
        telemetry.addData("Hang Arm Current Position",arm.hangArm.getCurrentPosition());
        if(keybind.poll("engage_hang")){
            Hanging=true;
        }else if(keybind.poll("disengage_hang")){
            Hanging=false;
        }
        if(keybind.poll("hang_change_direction_right")){
            HangRight=true;
        }else if(keybind.poll("hang_change_direction_left")){
            HangRight=false;
        }

        if(Hanging==false) {
            if (keybind.poll("hang_rotation_ccw")) {
                arm.rotateHangArm(RobotArm.Direction.COUNTER_CLOCKWISE, 1);
                telemetry.addData("Hang Arm Case", "CounterClock");
            } else if (keybind.poll("hang_rotation_cw")) {
                arm.rotateHangArm(RobotArm.Direction.CLOCKWISE, 1);
                telemetry.addData("Hang Arm Case", "Clock");
            } else {
                arm.rotateHangArm(RobotArm.Direction.BRAKE, 1);
                telemetry.addData("Hang Arm Case", "Brake");
            }
        }else{
            if(HangRight) {
                arm.rotateHangArm(RobotArm.Direction.COUNTER_CLOCKWISE, 1);
                telemetry.addData("Hang Arm Case", "Clock");
            }else{
                arm.rotateHangArm(RobotArm.Direction.CLOCKWISE, 1);
                telemetry.addData("Hang Arm Case", "Clock");
            }
        }
        telemetry.addData("Hang Arm Mode",arm.hangArm.getMode());

        if (keybind.poll("claw_open")) {
            arm.setRightClawPosition(0.57);
            arm.setLeftClawPosition(0.43);
        } else if (keybind.poll("claw_close")) {
            arm.setRightClawPosition(0);
            arm.setLeftClawPosition(1);
        } else {
            arm.setRightClawPosition(0.51);
            arm.setLeftClawPosition(0.49);
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

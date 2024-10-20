package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

class Keybind {

    private static final float DEADZONE = .1f;
    public enum Input {
        GAMEPAD_1_A,
        GAMEPAD_1_B,
        GAMEPAD_1_X,
        GAMEPAD_1_Y,
        GAMEPAD_1_DPAD_UP,
        GAMEPAD_1_DPAD_DOWN,
        GAMEPAD_1_DPAD_LEFT,
        GAMEPAD_1_DPAD_RIGHT,
        GAMEPAD_1_BACK,
        GAMEPAD_1_GUIDE,
        GAMEPAD_1_LEFT_BUMPER,
        GAMEPAD_1_RIGHT_BUMPER,
        GAMEPAD_1_RIGHT_STICK_BUTTON,
        GAMEPAD_1_LEFT_STICK_BUTTON,
        GAMEPAD_1_START,
        GAMEPAD_1_RIGHT_STICK_X,
        GAMEPAD_1_RIGHT_STICK_Y,
        GAMEPAD_1_LEFT_STICK_X,
        GAMEPAD_1_LEFT_STICK_Y,
        GAMEPAD_1_RIGHT_TRIGGER,
        GAMEPAD_1_LEFT_TRIGGER,
        GAMEPAD_2_A,
        GAMEPAD_2_B,
        GAMEPAD_2_X,
        GAMEPAD_2_Y,
        GAMEPAD_2_DPAD_UP,
        GAMEPAD_2_DPAD_DOWN,
        GAMEPAD_2_DPAD_LEFT,
        GAMEPAD_2_DPAD_RIGHT,
        GAMEPAD_2_BACK,
        GAMEPAD_2_GUIDE,
        GAMEPAD_2_LEFT_BUMPER,
        GAMEPAD_2_RIGHT_BUMPER,
        GAMEPAD_2_RIGHT_STICK_BUTTON,
        GAMEPAD_2_LEFT_STICK_BUTTON,
        GAMEPAD_2_START,
        GAMEPAD_2_RIGHT_STICK_X,
        GAMEPAD_2_RIGHT_STICK_Y,
        GAMEPAD_2_LEFT_STICK_X,
        GAMEPAD_2_LEFT_STICK_Y,
        GAMEPAD_2_RIGHT_TRIGGER,
        GAMEPAD_2_LEFT_TRIGGER,
        INVALID_INPUT,
        INVALID_ACTION
    }


    private HashMap<String, Input> keybinding;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    public Keybind(Gamepad gamepad1, Gamepad gamepad2) {
        this.keybinding = new HashMap<>();
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    private Input get(String action) {
        Input input = this.keybinding.get(action);
        if (input != null) {
            return input;
        }
            return Input.INVALID_ACTION;
    }

    //This method will add an action to the keybinding and return true if the value was updated.
    private boolean addOrUpdate(String action, Input input) {
        if (this.keybinding.containsKey(action)) {
            this.keybinding.replace(action, input);
            return true;
        } else {
            this.keybinding.put(action, input);
            return false;
        }
    }

    /*

    The buttons, analog sticks, and triggers are represented a public member variables that can be
    read from or written to directly.

    Analog sticks are represented as floats that range from -1.0 to +1.0. They will be 0.0 while at
    rest. The horizontal axis is labeled x, and the vertical axis is labeled y.

    Triggers are represented as floats that range from 0.0 to 1.0. They will be at 0.0 while at rest.
     */
    public float pollValue(String action) {
        Input input = get(action);
        switch (input) {
            case GAMEPAD_1_A: return gamepad1.a ? 0f : 1f;
            case GAMEPAD_1_B: return gamepad1.b ? 0f : 1f;
            case GAMEPAD_1_X: return gamepad1.x ? 0f : 1f;
            case GAMEPAD_1_Y: return gamepad1.y ? 0f : 1f;
            case GAMEPAD_1_DPAD_UP: return gamepad1.dpad_up ? 0f : 1f;
            case GAMEPAD_1_DPAD_DOWN: return gamepad1.dpad_down ? 0f : 1f;
            case GAMEPAD_1_DPAD_LEFT: return gamepad1.dpad_left ? 0f : 1f;
            case GAMEPAD_1_DPAD_RIGHT: return gamepad1.dpad_right ? 0f : 1f;
            case GAMEPAD_1_BACK: return gamepad1.back ? 0f : 1f;
            case GAMEPAD_1_GUIDE: return gamepad1.guide ? 0f : 1f;
            case GAMEPAD_1_LEFT_BUMPER: return gamepad1.left_bumper ? 0f : 1f;
            case GAMEPAD_1_RIGHT_BUMPER: return gamepad1.right_bumper ? 0f : 1f;
            case GAMEPAD_1_RIGHT_STICK_BUTTON: return gamepad1.right_stick_button ? 0f : 1f;
            case GAMEPAD_1_LEFT_STICK_BUTTON: return gamepad1.left_stick_button ? 0f : 1f;
            case GAMEPAD_1_START: return gamepad1.start ? 0f : 1f;
            case GAMEPAD_1_RIGHT_STICK_X: return gamepad1.right_stick_x > DEADZONE ? gamepad1.right_stick_x : 0f;
            case GAMEPAD_1_RIGHT_STICK_Y: return gamepad1.right_stick_y > DEADZONE ? gamepad1.right_stick_y : 0f;
            case GAMEPAD_1_LEFT_STICK_X: return gamepad1.left_stick_x > DEADZONE ? gamepad1.left_stick_x : 0f;
            case GAMEPAD_1_LEFT_STICK_Y: return gamepad1.left_stick_y > DEADZONE ? gamepad1.left_stick_y : 0f;
            case GAMEPAD_1_RIGHT_TRIGGER: return gamepad1.right_trigger > DEADZONE ? gamepad1.right_trigger : 0f;
            case GAMEPAD_1_LEFT_TRIGGER: return gamepad1.left_trigger > DEADZONE ? gamepad1.left_trigger : 0f;

            case GAMEPAD_2_A: return gamepad2.a ? 0f : 1f;
            case GAMEPAD_2_B: return gamepad2.b ? 0f : 1f;
            case GAMEPAD_2_X: return gamepad2.x ? 0f : 1f;
            case GAMEPAD_2_Y: return gamepad2.y ? 0f : 1f;
            case GAMEPAD_2_DPAD_UP: return gamepad2.dpad_up ? 0f : 1f;
            case GAMEPAD_2_DPAD_DOWN: return gamepad2.dpad_down ? 0f : 1f;
            case GAMEPAD_2_DPAD_LEFT: return gamepad2.dpad_left ? 0f : 1f;
            case GAMEPAD_2_DPAD_RIGHT: return gamepad2.dpad_right ? 0f : 1f;
            case GAMEPAD_2_BACK: return gamepad2.back ? 0f : 1f;
            case GAMEPAD_2_GUIDE: return gamepad2.guide ? 0f : 1f;
            case GAMEPAD_2_LEFT_BUMPER: return gamepad2.left_bumper ? 0f : 1f;
            case GAMEPAD_2_RIGHT_BUMPER: return gamepad2.right_bumper ? 0f : 1f;
            case GAMEPAD_2_RIGHT_STICK_BUTTON: return gamepad2.right_stick_button ? 0f : 1f;
            case GAMEPAD_2_LEFT_STICK_BUTTON: return gamepad2.left_stick_button ? 0f : 1f;
            case GAMEPAD_2_START: return gamepad2.start ? 0f : 1f;
            case GAMEPAD_2_RIGHT_STICK_X: return gamepad2.right_stick_x > DEADZONE ? gamepad2.right_stick_x : 0f;
            case GAMEPAD_2_RIGHT_STICK_Y: return gamepad2.right_stick_y > DEADZONE ? gamepad2.right_stick_y : 0f;
            case GAMEPAD_2_LEFT_STICK_X: return gamepad2.left_stick_x > DEADZONE ? gamepad2.left_stick_x : 0f;
            case GAMEPAD_2_LEFT_STICK_Y: return gamepad2.left_stick_y > DEADZONE ? gamepad2.left_stick_y : 0f;
            case GAMEPAD_2_RIGHT_TRIGGER: return gamepad2.right_trigger > DEADZONE ? gamepad2.right_trigger : 0f;
            case GAMEPAD_2_LEFT_TRIGGER: return gamepad2.left_trigger > DEADZONE ? gamepad2.left_trigger : 0f;

            case INVALID_INPUT: return -2f;
            case INVALID_ACTION: return -3f;
        }
        return -5f;
    }

    public boolean poll(String action) {
        Input input = get(action);
        switch (input) {
            case GAMEPAD_1_A: return gamepad1.a;
            case GAMEPAD_1_B: return gamepad1.b;
            case GAMEPAD_1_X: return gamepad1.x;
            case GAMEPAD_1_Y: return gamepad1.y;
            case GAMEPAD_1_DPAD_UP: return gamepad1.dpad_up;
            case GAMEPAD_1_DPAD_DOWN: return gamepad1.dpad_down;
            case GAMEPAD_1_DPAD_LEFT: return gamepad1.dpad_left;
            case GAMEPAD_1_DPAD_RIGHT: return gamepad1.dpad_right;
            case GAMEPAD_1_BACK: return gamepad1.back;
            case GAMEPAD_1_GUIDE: return gamepad1.guide;
            case GAMEPAD_1_LEFT_BUMPER: return gamepad1.left_bumper;
            case GAMEPAD_1_RIGHT_BUMPER: return gamepad1.right_bumper;
            case GAMEPAD_1_RIGHT_STICK_BUTTON: return gamepad1.right_stick_button;
            case GAMEPAD_1_LEFT_STICK_BUTTON: return gamepad1.left_stick_button;
            case GAMEPAD_1_START: return gamepad1.start;
            case GAMEPAD_1_RIGHT_STICK_X: return gamepad1.right_stick_x > DEADZONE;
            case GAMEPAD_1_RIGHT_STICK_Y: return gamepad1.right_stick_y > DEADZONE;
            case GAMEPAD_1_LEFT_STICK_X: return gamepad1.left_stick_x > DEADZONE;
            case GAMEPAD_1_LEFT_STICK_Y: return gamepad1.left_stick_y > DEADZONE;
            case GAMEPAD_1_RIGHT_TRIGGER: return gamepad1.right_trigger > DEADZONE;
            case GAMEPAD_1_LEFT_TRIGGER: return gamepad1.left_trigger > DEADZONE;

            case GAMEPAD_2_A: return gamepad2.a;
            case GAMEPAD_2_B: return gamepad2.b;
            case GAMEPAD_2_X: return gamepad2.x;
            case GAMEPAD_2_Y: return gamepad2.y;
            case GAMEPAD_2_DPAD_UP: return gamepad2.dpad_up;
            case GAMEPAD_2_DPAD_DOWN: return gamepad2.dpad_down;
            case GAMEPAD_2_DPAD_LEFT: return gamepad2.dpad_left;
            case GAMEPAD_2_DPAD_RIGHT: return gamepad2.dpad_right;
            case GAMEPAD_2_BACK: return gamepad2.back;
            case GAMEPAD_2_GUIDE: return gamepad2.guide;
            case GAMEPAD_2_LEFT_BUMPER: return gamepad2.left_bumper;
            case GAMEPAD_2_RIGHT_BUMPER: return gamepad2.right_bumper;
            case GAMEPAD_2_RIGHT_STICK_BUTTON: return gamepad2.right_stick_button;
            case GAMEPAD_2_LEFT_STICK_BUTTON: return gamepad2.left_stick_button;
            case GAMEPAD_2_START: return gamepad2.start;
            case GAMEPAD_2_RIGHT_STICK_X: return gamepad2.right_stick_x > DEADZONE;
            case GAMEPAD_2_RIGHT_STICK_Y: return gamepad2.right_stick_y > DEADZONE;
            case GAMEPAD_2_LEFT_STICK_X: return gamepad2.left_stick_x > DEADZONE;
            case GAMEPAD_2_LEFT_STICK_Y: return gamepad2.left_stick_y > DEADZONE;
            case GAMEPAD_2_RIGHT_TRIGGER: return gamepad2.right_trigger > DEADZONE;
            case GAMEPAD_2_LEFT_TRIGGER: return gamepad2.left_trigger > DEADZONE;

            case INVALID_INPUT:
            case INVALID_ACTION: return false;
        }
        return false;
    }
}
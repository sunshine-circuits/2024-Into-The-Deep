package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import java.util.HashMap;

public class RobotConfig {
    public enum RobotConfig {
        LM0 (new HardwareMapObject("FRMotor", DcMotor.class))
    }

    private HashMap<String, HardwareDevice> hardwareConfig = new HashMap<>();

    public RobotConfig(HardwareMapObject... hardwareObject) {

    }

    private class HardwareMapObject {
        String name;
        Class device;
        private HardwareMapObject(String name, Class deviceClass) {
            this.name = name;
            this.device = deviceClass;
        }

        private static HardwareMapObject hardwareMapObjectFactory(String name, Class deviceClass) {

        }
    }
}

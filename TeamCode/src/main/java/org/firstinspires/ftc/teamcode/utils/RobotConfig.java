package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.utils.HardwareMapObject.hardwareMapObjectFactory;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

public class RobotConfig {
    public enum Config {
        LM0(hardwareMapObjectFactory("FRMotor", DcMotor.class),
                hardwareMapObjectFactory("BRMotor", DcMotor.class),
                hardwareMapObjectFactory("FLMotor", DcMotor.class),
                hardwareMapObjectFactory("BLMotor", DcMotor.class),
                hardwareMapObjectFactory("ArmJointMotor", DcMotor.class),
                hardwareMapObjectFactory("ArmExtendMotor", DcMotor.class),
                hardwareMapObjectFactory("LeftServo", CRServo.class),
                hardwareMapObjectFactory("RightServo", CRServo.class)
        );


        /** @noinspection rawtypes*/
        private final HashMap<String, Class> config;
        private HashMap<String, HardwareDevice> hardwareConfig;

        Config(HardwareMapObject... hardwareObject) {
            this.config = new HashMap<>();
            for (HardwareMapObject hw : hardwareObject) {
                config.put(hw.name, hw.device);
            }
        }

        public void initConfig(HardwareMap map) {
            this.hardwareConfig = new HashMap<>();
            this.config.forEach((name, device) -> this.hardwareConfig.put(name, (HardwareDevice) map.get(device, name)));
        }

        public HardwareDevice get(String key) {
            if (null == this.hardwareConfig) {
                return null;
            }
            return this.hardwareConfig.get(key);
        }
    }


}

/** @noinspection rawtypes*/
class HardwareMapObject {
    String name;
    Class device;
    private HardwareMapObject(String name, Class deviceClass) {
        this.name = name;
        this.device = deviceClass;
    }

    static HardwareMapObject hardwareMapObjectFactory(String name, Class deviceClass) {
        return new HardwareMapObject(name, deviceClass);
    }
}


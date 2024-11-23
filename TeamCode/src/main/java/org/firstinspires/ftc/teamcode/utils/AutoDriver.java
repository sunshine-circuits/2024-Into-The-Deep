package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;

import android.util.Size;

import com.qualcomm.ftccommon.configuration.RobotConfigFile;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AutoDriver extends InchAutoParent{

    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor rearRightMotor;
    private DcMotor rearLeftMotor;

    AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            //These function and the following functions can customize the processor
            .setDrawAxes(true)//This tells the Processor to draw axis
            .setDrawTagOutline(true)//this draws the outline of the detected tag
            .setDrawTagID(true)//this puts the id of the tag in the middle of the tag
            .setDrawCubeProjection(true)//this draws a cube in front of the tag
            .build();//This is the final build function which builds what all functions have happened
    VisionPortal visionPortal = new VisionPortal.Builder()
            .addProcessor(tagProcessor)//this adds a processor
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .setCameraResolution(new Size(640, 480))
            .build();
    private Coordinate myLocation;

    public AutoDriver(RobotConfig.Config config){
        this.frontRightMotor = (DcMotor)config.get("FRMotor");
        this.rearRightMotor = (DcMotor)config.get("BRMotor");
        this.frontLeftMotor = (DcMotor)config.get("FLMotor");
        this.rearLeftMotor = (DcMotor)config.get("BLMotor");
    }

    public void drive(int FR, int FL, int BR, int BL, double pow){
        int FRtargetpos=frontRightMotor.getCurrentPosition()-(FR);
        int FLtargetpos=frontLeftMotor.getCurrentPosition()+(FL);
        int BRtargetpos=rearRightMotor.getCurrentPosition()-(BR);
        int BLtargetpos=rearLeftMotor.getCurrentPosition()+(BL);

        frontRightMotor.setTargetPosition(FRtargetpos);
        frontLeftMotor.setTargetPosition(FLtargetpos);
        rearRightMotor.setTargetPosition(BRtargetpos);
        rearLeftMotor.setTargetPosition(BLtargetpos);

        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        rearRightMotor.setPower(0);
        rearLeftMotor.setPower(0);
    }

    public void driveDistance(double distance, double angle, double pow){
        double pulses = inchesToPulses(distance);
        int pulsesInt = (int)pulses;
        drive(pulsesInt,pulsesInt,pulsesInt,pulsesInt,pow);
    }

    private double inchesToPulses(double Inches){
        return 537.7*(Inches/((104*Math.PI)/25.4));
    }

    public void rotate(double angle, double pow){
        double pulses = rotationsToPulses(angle-myLocation.yawRotation);
        int pulsesInt = (int)pulses;
        drive(-pulsesInt,pulsesInt,-pulsesInt,pulsesInt,pow);
    }
    private double rotationsToPulses(double degrees){
        return ((degrees/90)*930);
    }

    //TODO: Add AprilTag processing here and also add basic flow logic for pathing
    private void ProcessAprilTag(AprilTagProcessor tagProcessor)
    {
        if (tagProcessor.getDetections().size() > 0)
        {
            //basically, if there is at least one tag detected
            AprilTagDetection tag = tagProcessor.getDetections().get(0);
            //basically, 'tag' is the first tag the bot detects
            /*you can put tag.ftcPose to get a whole lot of info
            * tag.ftcPose.x gives the tag center position in x relative to the bot
            * tag.ftcPose.y gives the tag center position in y relative to the bot
            * tag.ftcPose.yaw gives the tag rotation relative to the bot
            * you can guess what the rest does!*/

        }
    }

}

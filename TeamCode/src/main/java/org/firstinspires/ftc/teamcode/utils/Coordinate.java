package org.firstinspires.ftc.teamcode.utils;

public class Coordinate {
    //definitions of the positions and rotation
    public double xPosition;
    public double yPosition;
    public double zPosition;
    public double yawRotation;
    // constructor
    Coordinate(double xPos, double yPos, double zPos, double yawRot) {
        xPosition = xPos;
        yPosition = yPos;
        zPosition = zPos;
        yawRotation = yawRot;

    }
    /*
    Coordinates of the AprilTags (Formatted to the defined field coordinate system):
    11: Coordinate(24,0,6,0);
    12: Coordinate(0,72,6,90);
    13: Coordinate(24,144,6,180);
    14: Coordinate(120,144,6,180);
    15: Coordinate(144,72,6,270);
    16: Coordinate(120,0,6,0);
     */
}

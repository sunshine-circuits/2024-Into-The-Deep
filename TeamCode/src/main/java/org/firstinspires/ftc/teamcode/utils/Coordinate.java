package org.firstinspires.ftc.teamcode.utils;

public class Coordinate {
    //definitions of the positions and rotation
    public float xPosition;
    public float yPosition;
    public float zPosition;
    public float yawRotation;
    // constructor
    Coordinate(float xPos, float yPos, float zPos, float yawRot) {
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

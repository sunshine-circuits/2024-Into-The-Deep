package org.firstinspires.ftc.teamcode.utils;

public enum DeepTags {
    TAG11(24,0,6,0),
    TAG12(0,72,6,90),
    TAG13(24,144,6,180),
    TAG14(120,144,6,180),
    TAG15(144,72,6,270),
    TAG16(120,0,6,0);
    public Coordinate cords;
    DeepTags(float xPos, float yPos, float zPos, float yawRot){
        this.cords = new Coordinate(xPos,yPos,zPos,yawRot);
    }


}

package org.firstinspires.ftc.teamcode.utils;

public enum Grid {
    BLUESAMPLECOLOR(0, 0, 0, 0), //Blue Observation Corner
    BLUEBASKETCORNER(0, 144, 0, 0), // Blue Basket Corner
    REDBASKETCORNER(144, 0, 0, 0), //Red Basket Corner
    REDSAMPLECORNER(144, 144, 0, 0), //Red Observation Corner
    SUBBLUESAMPLECORNER(48, 56, 0, 0), //SUBMERSIBLE ZONE FOR THE FOUR BELOW
    SUBREDSAMPLECORNER(96, 88, 0, 0),
    SUBBLUEBASKETCORNER(48, 88, 0, 0),
    SubmersiveRedBasketCorner(96, 56, 0, 0),
    BlueLowBasket(0, 144, 25.75, 0),
    BlueHighBasket(0, 144, 43, 0),
    RedLowBasket(144, 0, 25.75, 0),
    RedHighBakset(144, 0, 43, 0),
    TAG11(24, 0, 6, 0),
    TAG12(0, 72, 6, 90),
    TAG13(24, 144, 6, 180),
    TAG14(120, 144, 6, 180),
    TAG15(144, 72, 6, 270),
    TAG16(120, 0, 6, 0);

    public final Coordinate coord;

    Grid(double xPos, double yPos, double zPos, double yaw) {
        this.coord = new Coordinate(xPos, yPos, zPos, yaw);
    }
}


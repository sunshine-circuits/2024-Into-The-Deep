package org.firstinspires.ftc.teamcode.utils;

public class Grid {

    public enum GridPos {
        BLUESAMPLECOLOR(0, 0, 0), //Blue Observation Corner
        BLUEBASKETCORNER(0, 144, 0), // Blue Basket Corner
        REDBASKETCORNER(144, 0, 0), //Red Basket Corner
        REDSAMPLECORNER(144, 144, 0), //Red Observation Corner
        SUBBLUESAMPLECORNER(48, 56, 0), //SUBMERSIBLE ZONE FOR THE FOUR BELOW
        SUBREDSAMPLECORNER(96, 88, 0),
        SUBBLUEBASKETCORNER(48, 88, 0),
        SUBREDBASKETCORNER(96, 56, 0),
        BLUELOWBASKET(0, 144, 25.75),
        BLUEHIGHBASKET(0, 144, 43),
        REDLOWBASKET(144, 0, 25.75),
        REDHIGHBASKET(144, 0, 43);




        public Coordinate coords;

        GridPos(double xPos, double yPos, double zPos) {
            coords = new Coordinate(xPos,yPos,zPos,0);
        }
    }
}

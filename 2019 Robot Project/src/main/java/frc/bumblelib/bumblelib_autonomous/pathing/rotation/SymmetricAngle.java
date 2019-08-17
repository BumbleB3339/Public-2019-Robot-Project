package frc.bumblelib.bumblelib_autonomous.pathing.rotation;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.bumblelib.bumblelib_autonomous.pathing.path_point.SymmetricPathPoint;

public class SymmetricAngle extends Angle {

    private int redRightValue, redLeftValue, blueRightValue, blueLeftValue;

    public SymmetricAngle(int redRightValue, int redLeftValue, int blueRightValue, int blueLeftValue) {
        this.redRightValue = redRightValue;
        this.redLeftValue = redLeftValue;
        this.blueRightValue = blueRightValue;
        this.blueLeftValue = blueLeftValue;
    }
    
    /**
     * Positive angle outside.
     * @param redValue
     * @param blueValue
     */
    public SymmetricAngle(int redValue, int blueValue) {
        this(-redValue, redValue, -blueValue, blueValue);
    }

    /**
     * Positive angle outside.
     * @param value
     */
    public SymmetricAngle(int value) {
        this(value, value);
    }

    public SymmetricAngle(SymmetricPathPoint pathPoint) {
        this((int) pathPoint.getBumbleWaypoint(Alliance.RED, Side.RIGHT).getAngle(),
        (int) pathPoint.getBumbleWaypoint(Alliance.RED, Side.LEFT).getAngle(),
        (int) pathPoint.getBumbleWaypoint(Alliance.BLUE, Side.RIGHT).getAngle(),
        (int) pathPoint.getBumbleWaypoint(Alliance.BLUE, Side.LEFT).getAngle());
    }

    public void adjustRed(Side side, double dTheta) {
        switch (side) {
            case RIGHT:
                redRightValue += dTheta;
                break;
            case LEFT:
                redLeftValue += dTheta;
                break;
        }
    }

    public void adjustBlue(Side side, double dTheta) {
        switch (side) {
            case RIGHT:
                blueRightValue += dTheta;
                break;
            case LEFT:
                blueLeftValue += dTheta;
                break;
        }
    }

    public void adjustBoth(Side side, double dTheta) {
        switch (side) {
            case RIGHT:
                redRightValue += dTheta;
                blueRightValue += dTheta;
                break;
            case LEFT:
                redLeftValue += dTheta;
                blueLeftValue += dTheta;
                break;
        }
    }

    @Override
    public int getValue(Alliance alliance, Side side) {
        switch (alliance) {
            case RED:
                return side == Side.RIGHT ? redRightValue : redLeftValue;
            case BLUE:
                return side == Side.RIGHT ? blueRightValue : blueLeftValue;
        }
        return -3339;
    }
}
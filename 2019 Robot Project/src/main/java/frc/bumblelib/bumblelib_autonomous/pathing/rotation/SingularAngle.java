package frc.bumblelib.bumblelib_autonomous.pathing.rotation;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.bumblelib.bumblelib_autonomous.pathing.path_point.SingularPathPoint;

public class SingularAngle extends Angle {

    private int redValue, blueValue;

    public SingularAngle(int redValue, int blueValue) {
        this.redValue = redValue;
        this.blueValue = blueValue;
    }

    public SingularAngle(int value) {
        this(value, value);
    }

    public SingularAngle(SingularPathPoint pathPoint) {
        this((int) pathPoint.getBumbleWaypoint(Alliance.RED).getAngle(), (int) pathPoint.getBumbleWaypoint(Alliance.BLUE).getAngle());
    }

    public void adjustRed(int dTheta) {
        redValue += dTheta;
    }

    public void adjustBlue(int dTheta) {
        blueValue += dTheta;
    }

    public void adjustBoth(int dTheta) {
        redValue += dTheta;
        blueValue += dTheta;
    }

    @Override
	public int getValue(Alliance alliance, Side side) {
		return alliance == Alliance.RED ? redValue : blueValue;
    }
}
package frc.bumblelib.bumblelib_autonomous.pathing;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;

public class AutonomousSettings {

    private static Alliance alliance = Alliance.RED;
    private static Side side = Side.RIGHT;

    public static Alliance getAlliance() {
        return alliance;
    }

    /**
     * Gets the side.
     * @return the side.
     */
    public static Side getSide() {
        return side;
    }

    /**
     * @param side the side to set
     */
    public static void setSide(Side side) {
        AutonomousSettings.side = side;
    }

    /**
     * @param alliance the alliance to set
     */
    public static void setAlliance(Alliance alliance) {
        AutonomousSettings.alliance = alliance;
    }
}

package frc.bumblelib.bumblelib_autonomous.pathing.enums;

import edu.wpi.first.wpilibj.DriverStation;

public enum Alliance {
    RED, BLUE;

    public static Alliance getDriverStationAlliance(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Red ? RED : BLUE;
    }
}

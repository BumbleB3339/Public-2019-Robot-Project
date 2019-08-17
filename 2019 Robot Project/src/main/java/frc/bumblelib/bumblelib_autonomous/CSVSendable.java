package frc.bumblelib.bumblelib_autonomous;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import jaci.pathfinder.Trajectory;

public interface CSVSendable {

    public String getName();
    public String getNameTemplate(Alliance alliance, Side side);
    public String getDescription(Alliance alliance, Side side);
    public Trajectory generateTrajectory(Alliance alliance, Side side);

}
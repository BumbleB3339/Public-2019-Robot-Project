package frc.bumblelib.bumblelib_autonomous.pathing.path;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Direction;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.bumblelib.bumblelib_autonomous.pathing.path_point.PathPoint;
import frc.bumblelib.util.PIDPreset;
import frc.robot.subsystems.Drivetrain.Gear;

public class SymmetricPath extends Path {

    public SymmetricPath(String pathName, Gear gear, double maxVelocity, Direction direction, PIDPreset pidPreset) {
        super(pathName, gear, maxVelocity, direction, pidPreset);
    }

    protected void add(PathPoint pathPoint) {
        pathPoints.add(pathPoint);
    }

    /**
     * Generates the name for a CSV file according to 'pathName-COLOR-SIDE'
     * template.
     */
    @Override
    public String getNameTemplate(Alliance alliance, Side side) {
        return super.getName() + "-" + alliance + "-" + side + ".csv";
    }
}

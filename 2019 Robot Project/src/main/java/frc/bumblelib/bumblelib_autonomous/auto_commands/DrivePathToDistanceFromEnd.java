package frc.bumblelib.bumblelib_autonomous.auto_commands;

import frc.bumblelib.bumblelib_autonomous.pathing.path.Path;

public class DrivePathToDistanceFromEnd extends DrivePath {

    private double distanceFromEnd;

    public DrivePathToDistanceFromEnd(Path path, double distanceFromEnd) {
        super(path);

        this.distanceFromEnd = distanceFromEnd;
    }

    @Override
    protected boolean isFinished() {
        return super.isFinished() || super.follower.getDistanceToEnd() <= distanceFromEnd;
    }

    @Override
    protected void end() {
        super.iter.stop();
        follower.endPath();
    }
}
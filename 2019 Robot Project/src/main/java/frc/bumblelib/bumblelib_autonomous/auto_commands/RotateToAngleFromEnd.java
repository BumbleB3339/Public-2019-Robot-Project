package frc.bumblelib.bumblelib_autonomous.auto_commands;

import frc.bumblelib.bumblelib_autonomous.pathing.rotation.Rotation;

public class RotateToAngleFromEnd extends Rotate {

    private double angleFromEnd;

    public RotateToAngleFromEnd(Rotation rotation, double angleFromEnd) {
        super(rotation);
        this.angleFromEnd = angleFromEnd;
    }

    @Override
    protected boolean isFinished() {
        return super.isFinished() || Math.abs(super.rotation.getRotationAngle() - super.rotation.getCurrentAngle()) <= angleFromEnd;
    }

}
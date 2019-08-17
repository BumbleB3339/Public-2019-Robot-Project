package frc.bumblelib.util.hardware;

import jaci.pathfinder.Pathfinder;

public abstract class Gyro {
    protected double relativeOffset, permanentOffset = 0.0;
    protected boolean isRelativeActive = false;

    /**
     * 
     * @return
     *      The current yaw returned by the sensor, positive counter-clockwise (CCW).
     */
    public abstract double getRawYaw();

    public abstract void reset();

    public abstract void reCalibrateAxis();

    public void setOffset(double offset) {
        permanentOffset = offset;
    }

    public double getYaw() {
        return Pathfinder.boundHalfDegrees(getRawYaw() + permanentOffset);
    }

    public void startRelativeMode() throws GyroRelativeModeException {
        if (isRelativeActive) {
            throw new GyroRelativeModeException("Gyro Already in Relative Mode!");
        }
        this.relativeOffset = getYaw();
        isRelativeActive = true;
    }

    /**
     * Starts relative mode and sets given parameter as heading.
     * @param initialHeading
     *          the initial heading of the robot after relative mode has begun.
     * @throws Exception
     */
    public void startRelativeMode(double initialHeading) throws GyroRelativeModeException {
        if (isRelativeActive) {
            throw new GyroRelativeModeException("Gyro Already in Relative Mode!");
        }
        this.relativeOffset = getYaw() - initialHeading;
        isRelativeActive = true;
    }

    /**
     * Get the yaw of the robot in relation to where the robot was when relative mode started, plus initial heading if defined.
     * @return
     * @throws Exception
     */
    public double getRelativeYaw() throws GyroRelativeModeException {
        if (!isRelativeActive) {
            throw new GyroRelativeModeException("Gyro Not in Relative Mode!");
        }
        return Pathfinder.boundHalfDegrees(this.getRawYaw() - this.relativeOffset);
    }

    public void endRelativeMode() {
        this.isRelativeActive = false;
    }

    private class GyroRelativeModeException extends Exception {
        private static final long serialVersionUID = 1L;

        public GyroRelativeModeException(String msg) {
            super(msg);
        }
    }
}

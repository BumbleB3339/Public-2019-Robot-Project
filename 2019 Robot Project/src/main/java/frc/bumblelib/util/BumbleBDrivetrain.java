package frc.bumblelib.util;

import frc.bumblelib.util.hardware.Gyro;

public interface BumbleBDrivetrain {
    
    public Gyro getGyro();
    public int getRightEncoderTicks();
    public int getLeftEncoderTicks();
    public double getRightVelocity();
    public double getLeftVelocity();
    public double getAverageVelocity();
    public void resetEncoders();
}

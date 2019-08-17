/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.util;

import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class VelocityKalmanFilter extends KalmanFilter {
    private double lastValue = -3339;
    private double lastFPGATimestamp = Timer.getFPGATimestamp();

    public VelocityKalmanFilter(int windowSize) {
        super(windowSize);
    }

    @Override
    public void addValue(double newValue) {
        double currentFPGATimestamp = Timer.getFPGATimestamp();

        if (lastValue != -3339) {
            super.addValue((newValue - lastValue) / (currentFPGATimestamp - lastFPGATimestamp));
        }

        lastFPGATimestamp = currentFPGATimestamp;
        lastValue = newValue;
    }
}

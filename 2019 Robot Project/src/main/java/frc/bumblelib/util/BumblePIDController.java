/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.util;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

/**
 * Add your docs here.
 */
public class BumblePIDController extends PIDController {

    private double lastUpdatedMinimumOutput = -3339;
    private double lastUpdatedMaximumOutput = -3339;
    private boolean isEnabled = false;

    private double lastUpdatedP = 0.0, lastUpdatedI = 0.0, lastUpdatedD = 0.0;

    public BumblePIDController(double Kp, double Ki, double Kd, PIDSource source, PIDOutput output) {
        super(Kp, Ki, Kd, source, output);
    }

    @Override
    public void setOutputRange(double minimumOutput, double maximumOutput) {
        if (minimumOutput != lastUpdatedMinimumOutput || maximumOutput != lastUpdatedMaximumOutput) {
            super.setOutputRange(minimumOutput, maximumOutput);
            lastUpdatedMaximumOutput = maximumOutput;
            lastUpdatedMinimumOutput = minimumOutput;
        }
    }

    @Override
    public void enable() {
        if (!isEnabled) {
            super.enable();
            isEnabled = true;
        } 
    }

    @Override
    public void disable() {
        if (isEnabled) {
            super.disable();
            isEnabled = false;
        } 
    }

    @Override
    public void setPID(double p, double i, double d) {
        if (p != lastUpdatedP || i != lastUpdatedI || d != lastUpdatedD) {
            super.setPID(p, i, d);
            lastUpdatedP = p;
            lastUpdatedI = i;
            lastUpdatedD = d;
        }
    }

    @Override
    public void setP(double p) {
        if (p != lastUpdatedP) {
            super.setP(p);
            lastUpdatedP = p;
        }
    }

    @Override
    public void setI(double i) {
        if (i != lastUpdatedI) {
            super.setI(i);
            lastUpdatedI = i;
        }
    }

    @Override
    public void setD(double d) {
        if (d != lastUpdatedD) {
            super.setD(d);
            lastUpdatedD = d;
        }
    }
}

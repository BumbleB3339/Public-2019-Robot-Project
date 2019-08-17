/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.util.maneuvers;

/**
 * Add your docs here.
 */
public interface PIDManeuver {
    
    public void init(double setpoint);

    public void execute();

    public boolean isFinished();
    
    public void end();

    public void initCalibration();

    public void executeCalibration();
}

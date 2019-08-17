/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import frc.robot.Robot;

public class FrontCoordinatedClimb extends CoordinatedClimb {
  public FrontCoordinatedClimb(double setpoint, boolean isBothSync, boolean disablePID) {
    super(Robot.m_frontClimb, Robot.m_rearClimb, setpoint, isBothSync, disablePID);
  } 

  public FrontCoordinatedClimb(double setpoint, boolean isBothSync){
    super(Robot.m_frontClimb, Robot.m_rearClimb, setpoint, isBothSync);
  }
}

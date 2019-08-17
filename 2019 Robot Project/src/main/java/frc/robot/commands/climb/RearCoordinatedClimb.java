/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import frc.robot.Robot;

public class RearCoordinatedClimb extends CoordinatedClimb {
  public RearCoordinatedClimb(double setpoint, boolean isBothSync, boolean disablePID) {
    super(Robot.m_rearClimb, Robot.m_frontClimb, setpoint, isBothSync, disablePID);
  }

  public RearCoordinatedClimb(double setpoint, boolean isBothSync) {
    super(Robot.m_rearClimb, Robot.m_frontClimb, setpoint, isBothSync);
  }
}

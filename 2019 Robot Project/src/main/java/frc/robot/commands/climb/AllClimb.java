/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AllClimb extends CommandGroup {

  public AllClimb(double height) {
    addParallel(new FrontCoordinatedClimb(height, true));
    addParallel(new RearCoordinatedClimb(height, true));
  }

  public AllClimb(double height, boolean disablePID) {
    addParallel(new FrontCoordinatedClimb(height, true, disablePID));
    addParallel(new RearCoordinatedClimb(height, true, disablePID));
  }

  public AllClimb(double height, boolean disablePID, boolean isBothSync) {
    addParallel(new FrontCoordinatedClimb(height, isBothSync, disablePID));
    addParallel(new RearCoordinatedClimb(height, isBothSync, disablePID));
  }
}

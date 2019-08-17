/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.operation.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.RobotGameStateManager;
import frc.robot.RobotGameStateManager.ClimbMode;

/**
 * Add your docs here.
 */
public class CommandToggleClimbMode extends InstantCommand {
  /**
   * Add your docs here.
   */
  public CommandToggleClimbMode() {
    super();
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (RobotGameStateManager.climbMode == ClimbMode.LEVEL2) {
      RobotGameStateManager.climbMode = ClimbMode.LEVEL3;
    } else {
      RobotGameStateManager.climbMode = ClimbMode.LEVEL2;
    }
  }

}

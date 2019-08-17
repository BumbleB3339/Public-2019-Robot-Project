/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.operation.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotGameState.FieldObjective;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameStateManager;

public class CommandSetFieldObjective extends Command {

  private FieldObjective fieldObjective;

  public CommandSetFieldObjective(FieldObjective fieldObjective) {
    this.fieldObjective = fieldObjective;
  }

  @Override
  protected void initialize() {
    RobotGameStateManager.nextGameState.fieldObjective = fieldObjective;
    
    if (RobotGameStateManager.currentGameState.robotAction == RobotAction.PRE_PLACEMENT ||
      RobotGameStateManager.currentGameState.robotAction == RobotAction.POST_PLACEMENT) {
      RobotGameStateManager.updateCurrentConfiguration();
    }
  }

  @Override
  protected boolean isFinished() {
    return true;
  }
}

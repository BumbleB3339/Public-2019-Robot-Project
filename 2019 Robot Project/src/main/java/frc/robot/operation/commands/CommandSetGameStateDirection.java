/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.operation.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotGameState.Direction;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameStateManager;

public class CommandSetGameStateDirection extends Command {
  private Direction direction;
  private static CommandFoldSystems foldSystems = new CommandFoldSystems();

  public CommandSetGameStateDirection(Direction direction) {
    this.direction = direction;
  }

  @Override
  protected void initialize() {
    RobotGameStateManager.nextGameState.direction = direction;

    if (RobotGameStateManager.currentGameState.robotAction == RobotAction.PRE_PLACEMENT) {
      RobotGameStateManager.updateCurrentConfiguration();
    } else if (RobotGameStateManager.currentGameState.robotAction == RobotAction.POST_PLACEMENT) {
      RobotGameStateManager.currentGameState.direction = direction;
      foldSystems.start();
    }
  }

  @Override
  protected boolean isFinished() {
    return true;
  }
}

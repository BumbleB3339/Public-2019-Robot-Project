/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.operation.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.StartCommand;
import frc.robot.RobotGameState.GamePiece;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameStateManager;

public class CommandSetGamePiece extends CommandGroup {

  public CommandSetGamePiece(GamePiece gamePiece) {
    addSequential(new InstantCommand(() -> {
      RobotGameStateManager.nextGameState.gamePiece = gamePiece;
      
      if (RobotGameStateManager.currentGameState.robotAction == RobotAction.PRE_PLACEMENT ||
        RobotGameStateManager.currentGameState.robotAction == RobotAction.POST_PLACEMENT) {
        RobotGameStateManager.updateCurrentConfiguration();
      }
    }));

    addSequential(new ConditionalCommand(new StartCommand(new CommandFeederCollect(false))){
      @Override
      protected boolean condition() {
        return RobotGameStateManager.currentGameState.robotAction == RobotAction.FEEDER_COLLECT;
      }
    });

    addSequential(new ConditionalCommand(new StartCommand(new CommandFloorCollect(false))){
      @Override
      protected boolean condition() {
        return RobotGameStateManager.currentGameState.robotAction == RobotAction.FLOOR_COLLECT;
      }
    });
  }
}

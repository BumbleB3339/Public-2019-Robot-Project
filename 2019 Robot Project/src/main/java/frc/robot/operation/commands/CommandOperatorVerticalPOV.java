/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.operation.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotGameState.CargoShipFace;
import frc.robot.RobotGameState.FieldObjective;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameState.RocketPlacementHeight;
import frc.robot.RobotGameStateManager;

public class CommandOperatorVerticalPOV extends Command {

  public enum POV_VerticalDirection {
    UP, DOWN;
  }

  private POV_VerticalDirection verticalDirection;

  public CommandOperatorVerticalPOV(POV_VerticalDirection verticalDirection) {
    this.verticalDirection = verticalDirection;
  }

  @Override
  protected void initialize() {
    switch (verticalDirection) {
    case UP:
      if (true || RobotGameStateManager.nextGameState.fieldObjective == FieldObjective.ROCKET) {
        if (RobotGameStateManager.nextGameState.rocketPlacementHeight == RocketPlacementHeight.LEVEL1) {
          RobotGameStateManager.nextGameState.rocketPlacementHeight = RocketPlacementHeight.LEVEL2;
        } else {
          RobotGameStateManager.nextGameState.rocketPlacementHeight = RocketPlacementHeight.LEVEL3;
        }
      } else if (RobotGameStateManager.nextGameState.fieldObjective == FieldObjective.CARGOSHIP) {
        RobotGameStateManager.nextGameState.cargoShipFace = CargoShipFace.SIDE;
      }
      break;
    case DOWN:
      if (true || RobotGameStateManager.nextGameState.fieldObjective == FieldObjective.ROCKET) {
        if (RobotGameStateManager.nextGameState.rocketPlacementHeight == RocketPlacementHeight.LEVEL3) {
          RobotGameStateManager.nextGameState.rocketPlacementHeight = RocketPlacementHeight.LEVEL2;
        } else {
          RobotGameStateManager.nextGameState.rocketPlacementHeight = RocketPlacementHeight.LEVEL1;
        }
      } else if (RobotGameStateManager.nextGameState.fieldObjective == FieldObjective.CARGOSHIP) {
        RobotGameStateManager.nextGameState.cargoShipFace = CargoShipFace.FRONT;
      }
      break;
    }

    if (RobotGameStateManager.nextGameState.fieldObjective == FieldObjective.ROCKET && (RobotGameStateManager.currentGameState.robotAction == RobotAction.PRE_PLACEMENT
        || RobotGameStateManager.currentGameState.robotAction == RobotAction.POST_PLACEMENT)) {
      RobotGameStateManager.updateCurrentConfiguration();
    }
  } 

  @Override
  protected boolean isFinished() {
    return true;
  }
}

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
import frc.robot.Robot;
import frc.robot.RobotGameState.Direction;
import frc.robot.RobotGameState.FieldObjective;
import frc.robot.RobotGameState.GamePiece;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameState.RobotSystemState;
import frc.robot.RobotGameState.RocketPlacementHeight;
import frc.robot.RobotGameStateManager;
import frc.robot.commands.SetCurrentRobotSystemState;
import frc.robot.commands.WaitForTrue;
import frc.robot.commands.intake_arm.SetIntakeState;
import frc.robot.subsystems.IntakeArm.IntakeArmState;

public class CommandSetToPrePlacement extends CommandGroup {

  public CommandSetToPrePlacement() {
    requires(Robot.m_lift);
    requires(Robot.m_liftArm);
    requires(Robot.m_elementArm);
    requires(Robot.m_intakeArm);

    addParallel(new SetIntakeState(IntakeArmState.FOLDED, false));

    addSequential(new ConditionalCommand(new CommandFoldBeforePlacing()) {

      // should make room for intake room? if not, it means that it can fold
      // immidiately
      @Override
      protected boolean condition() {
        boolean level1rocket = RobotGameStateManager.nextGameState.fieldObjective == FieldObjective.ROCKET
            && RobotGameStateManager.nextGameState.direction == Direction.FORWARD
            && RobotGameStateManager.nextGameState.rocketPlacementHeight == RocketPlacementHeight.LEVEL1;
        boolean hatchCargoship = RobotGameStateManager.nextGameState.fieldObjective == FieldObjective.CARGOSHIP
            && RobotGameStateManager.nextGameState.gamePiece == GamePiece.HATCH_PANEL
            && RobotGameStateManager.nextGameState.direction == Direction.FORWARD;
        return !Robot.m_intakeArm.isApproximatelyFolded() && (level1rocket || hatchCargoship);
      }
    });

    addSequential(new InstantCommand(() -> {
      RobotGameStateManager.nextGameState.robotAction = RobotAction.PRE_PLACEMENT;
      RobotGameStateManager.updateCurrentConfiguration();
    }));
  }

  class CommandFoldBeforePlacing extends CommandGroup {
    public CommandFoldBeforePlacing() {
      addParallel(new SetCurrentRobotSystemState(RobotSystemState.FRONT_INTAKE_ARM_GAP_HIGH, false));
      addSequential(new WaitForTrue(() -> Robot.m_intakeArm.isApproximatelyFolded()));
    }
  }
}

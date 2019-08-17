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
import frc.bumblelib.bumblelib_autonomous.auto_commands.WaitAndThen;
import frc.robot.ControlledGamePieceDetector;
import frc.robot.ControlledGamePieceDetector.ControlledGamePiece;
import frc.robot.Robot;
import frc.robot.RobotGameState.Direction;
import frc.robot.RobotGameState.GamePiece;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameState.RobotSystemState;
import frc.robot.RobotGameStateManager;
import frc.robot.commands.SetCurrentRobotSystemState;
import frc.robot.commands.WaitForTrue;
import frc.robot.commands.cargo_handler.CommandStrongHoldCargo;
import frc.robot.commands.cargo_handler.StopCargoHandler;
import frc.robot.commands.intake_arm.SetIntakeState;
import frc.robot.subsystems.IntakeArm.IntakeArmState;

public class CommandFoldSystems extends CommandGroup {
  public static boolean isRunning = false;

  @Override
  protected void initialize() {
    isRunning = true;
  }

  public CommandFoldSystems() {
    requires(Robot.m_lift);
    requires(Robot.m_liftArm);
    requires(Robot.m_elementArm);
    requires(Robot.m_intakeArm);
    
    addParallel(new ConditionalCommand(new StartCommand(new CommandStrongHoldCargo())) {

      @Override
      protected boolean condition() {
        return RobotGameStateManager.currentGameState.robotAction == RobotAction.FLOOR_COLLECT
            && RobotGameStateManager.currentGameState.gamePiece == GamePiece.CARGO;
      }
    });

    // Folds intake arm only if the intake arm isn't folded.
    addSequential(new ConditionalCommand(new FoldIntakeWithoutCollision()) {

      @Override
      protected boolean condition() {
        // intake arm not folded and folding forward
        return RobotGameStateManager.currentGameState.direction == Direction.FORWARD
            && !Robot.m_intakeArm.isApproximatelyFolded();
      }
    });

    addParallel(new WaitAndThen(0.5, new StartCommand(new StopCargoHandler()), false));

    // on feeder and floor collect fold to current direction
    addSequential(new ConditionalCommand(new FoldToCurrent(), new InstantCommand(() -> {
      // on other states fold to next direction
      RobotGameStateManager.nextGameState.robotAction = RobotAction.FOLDED;
      RobotGameStateManager.updateCurrentConfiguration();
    })) {

      @Override
      protected boolean condition() {
        return RobotGameStateManager.currentGameState.robotAction == RobotAction.FEEDER_COLLECT
            || RobotGameStateManager.currentGameState.robotAction == RobotAction.FLOOR_COLLECT
            || RobotGameStateManager.currentGameState.robotAction == RobotAction.ABORT_CARGO_FLOOR_COLLECT;
      }
    });
  }

  class FoldIntakeWithoutCollision extends CommandGroup {
    public FoldIntakeWithoutCollision() {
      addParallel(new SetCurrentRobotSystemState(RobotSystemState.FRONT_INTAKE_ARM_GAP_HIGH, false));
      addParallel(new SetIntakeState(IntakeArmState.FOLDED, false));
      addSequential(new WaitForTrue(() -> Robot.m_intakeArm.isApproximatelyFolded()));
    }
  }

  class FoldToCurrent extends CommandGroup {
    public FoldToCurrent() {
      addSequential(new ConditionalCommand(new SetCurrentRobotSystemState(RobotSystemState.FOLDED, false),
          new SetCurrentRobotSystemState(RobotSystemState.FOLDED_WITH_CARGO, false)) {

        @Override
        protected boolean condition() {
          return ControlledGamePieceDetector.getControlledGamePiece() == ControlledGamePiece.HATCH_PANEL;
        }
      });

      addParallel(new WaitAndThen(0.2, new InstantCommand(() -> {
        RobotGameStateManager.currentGameState.robotAction = RobotAction.FOLDED;
        RobotGameStateManager.nextGameState.robotAction = RobotAction.FOLDED;
      })));
    }
  }

  @Override
  public void end() {
    isRunning = false;
  }

  @Override
  public void interrupted() {
    end();
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.sequences.sub_sequences;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.bumblelib.bumblelib_autonomous.auto_commands.DrivePathToDistanceFromEnd;
import frc.bumblelib.bumblelib_autonomous.auto_commands.Wait;
import frc.bumblelib.bumblelib_autonomous.auto_commands.WaitAndThen;
import frc.bumblelib.bumblelib_autonomous.pathing.AutonomousSettings;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.robot.Robot;
import frc.robot.RobotGameState.Direction;
import frc.robot.RobotGameState.FieldObjective;
import frc.robot.RobotGameState.GamePiece;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameState.RobotSystemState;
import frc.robot.RobotGameState.RocketPlacementHeight;
import frc.robot.RobotGameState.RocketSide;
import frc.robot.RobotGameStateManager;
import frc.robot.autonomous.commands.AutoPlaceOnRocket;
import frc.robot.autonomous.commands.CommandAutoAlignToTarget;
import frc.robot.autonomous.paths.Paths;
import frc.robot.commands.SetCurrentRobotSystemState;
import frc.robot.operation.commands.CommandSetFieldObjective;
import frc.robot.operation.commands.CommandSetGamePiece;
import frc.robot.profiles.InitProfiles;

public class PlatformCloseRocketFeederSequence extends CommandGroup {

  private static final double CLOSE_ROCKET_DISTANCE_FROM_END = 1.5;
  private static final double FEEDER_DISTANCE_FROM_END = 2.3;

  /**
   * Add your docs here.
   */
  public PlatformCloseRocketFeederSequence() {
    // Set logical state to rocket hatch placement.
    addSequential(new CommandSetGamePiece(GamePiece.HATCH_PANEL));
    addSequential(new CommandSetFieldObjective(FieldObjective.ROCKET));
    addSequential(new InstantCommand(() -> {
      RobotGameStateManager.nextGameState.rocketSide = AutonomousSettings.getSide() == Side.LEFT ? RocketSide.LEFT
          : RocketSide.RIGHT;
      RobotGameStateManager.nextGameState.rocketPlacementHeight = RocketPlacementHeight.LEVEL2;
      RobotGameStateManager.currentGameState.rocketPlacementHeight = RocketPlacementHeight.LEVEL2;
    }));

    addParallel(new WaitAndThen(0.5, new InstantCommand(() -> {
      Robot.m_pixyVision.LED.set(true);
      Robot.m_pixyVision.setEnabled(true);
    })));

    // Drive to rocket and set physical state to placement.
    addParallel(new WaitAndThen(0.0, new SetCurrentRobotSystemState(RobotSystemState.L2_HATCH_FOLDED, true)));
    // addSequential(new
    // DrivePathToDistanceFromEnd(Paths.farPlatformToCloseRocketPlacement,
    // CLOSE_ROCKET_DISTANCE_FROM_END));

    addSequential(new DrivePathToDistanceFromEnd(Paths.relativeToCloseRocket, CLOSE_ROCKET_DISTANCE_FROM_END));

    addSequential(new InstantCommand(() -> {
      RobotGameStateManager.currentGameState.robotAction = RobotAction.PRE_PLACEMENT;
    }));
    // Align to target and place hatch.
    addSequential(new ConditionalCommand(new AutoPlaceOnRocket(-InitProfiles.FIELD_PROFILE.closeRocketAngle()),
        new AutoPlaceOnRocket(InitProfiles.FIELD_PROFILE.closeRocketAngle())) {

      @Override
      public boolean condition() {
        return AutonomousSettings.getSide() == Side.RIGHT;
      }
    });

    addParallel(new ConditionalCommand(new WaitAndThen(0.4, new InstantCommand(() -> {
      RobotGameStateManager.currentGameState.direction = Direction.BACKWARD;

      RobotGameStateManager.currentGameState.robotAction = RobotAction.FEEDER_COLLECT;
      RobotGameStateManager.nextGameState.robotAction = RobotAction.FEEDER_COLLECT;
      RobotGameStateManager.currentGameState.robotSystemState = RobotSystemState.FEEDER_HATCH_START;
    })), new WaitAndThen(0.4, new InstantCommand(() -> {
      RobotGameStateManager.currentGameState.direction = Direction.BACKWARD;
      RobotGameStateManager.currentGameState.robotAction = RobotAction.FEEDER_COLLECT;
      RobotGameStateManager.nextGameState.robotAction = RobotAction.FEEDER_COLLECT;
      RobotGameStateManager.currentGameState.robotSystemState = RobotSystemState.FEEDER_HATCH_START;
    }))) {
      @Override
      public boolean condition() {
        return AutoPlaceOnRocket.aligned;
      }
    });

    addParallel(new WaitAndThen(0.5,
        new InstantCommand(() -> RobotGameStateManager.nextGameState.direction = Direction.BACKWARD)));

    // Drive to feeder and set physical state to collection.
    addSequential(new DrivePathToDistanceFromEnd(Paths.closeRocketToFeeder, FEEDER_DISTANCE_FROM_END));

    // Align to target and collect hatch.
    addSequential(new CommandAutoAlignToTarget());
    addParallel(new InstantCommand(() -> {
      Robot.m_hatchHandler.extendHoldSolenoid();
    }));

    addSequential(new Wait(0.3));
  }
}

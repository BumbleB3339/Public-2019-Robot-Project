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
import frc.robot.RobotGameState.Direction;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameState.RobotSystemState;
import frc.robot.RobotGameStateManager;
import frc.robot.commands.SetCurrentRobotSystemState;
import frc.robot.commands.intake_arm.IntakeArmSetPosition;
import frc.robot.operation.commands.CommandPulsedLED.PulseType;
import frc.robot.subsystems.IntakeArm.IntakeArmState;

public class CommandPrepareToClimb extends CommandGroup {
  /**
   * Add your docs here.
   */

  private final Direction PREPARE_TO_CLIMB_DIRECTION = Direction.FORWARD;

  public CommandPrepareToClimb() {
    addParallel(new StartCommand(new CommandPulsedLED(PulseType.SLOW)));
    addSequential(new InstantCommand(() -> {
      RobotGameStateManager.currentGameState.robotAction = RobotAction.CLIMB;
      RobotGameStateManager.currentGameState.direction = PREPARE_TO_CLIMB_DIRECTION;
      RobotGameStateManager.nextGameState.direction = Direction.FORWARD; // so that the camera looks forward
    }));

    addSequential(
        new ConditionalCommand(new SetCurrentRobotSystemState(RobotSystemState.FRONT_INTAKE_ARM_GAP_LOW, false),
            new SetCurrentRobotSystemState(RobotSystemState.FOLDED, false)) {

          @Override
          protected boolean condition() {
            return PREPARE_TO_CLIMB_DIRECTION == Direction.FORWARD;
          }
        });
    addSequential(new IntakeArmSetPosition(IntakeArmState.FOLDED));
  }
}

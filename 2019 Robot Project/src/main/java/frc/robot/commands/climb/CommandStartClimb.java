/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.RobotGameStateManager;
import frc.robot.RobotGameStateManager.ClimbMode;
import frc.robot.operation.commands.CommandPulsedLED;
import frc.robot.operation.commands.CommandPulsedLED.PulseType;

public class CommandStartClimb extends CommandGroup {
  public CommandStartClimb() {
    addParallel(new CommandPulsedLED(PulseType.SLOW, 0.0));
    addSequential(new ConditionalCommand(new CommandClimbLevel2()) {

      @Override
      protected boolean condition() {
        return RobotGameStateManager.climbMode == ClimbMode.LEVEL2;
      }
    });
    addSequential(new ConditionalCommand(new CommandClimbLevel3()) {

      @Override
      protected boolean condition() {
        return RobotGameStateManager.climbMode == ClimbMode.LEVEL3;
      }
    });
  }

  // With new drivetrain not moving logic
  /**
   * Add your docs here.
   */
  // public CommandStartClimb() {
  //   addSequential(new ConditionalCommand(
  //       new StartCommand(
  //           new CommandTimedControllerRumble(OI.driverController, RumbleType.kLeftRumble, RumbleLength.SHORT, 0.5)),
  //       new Climb()) {

  //     @Override
  //     protected boolean condition() {
  //       return Robot.m_drivetrain.isDrivetrainMoving();
  //     }
  //   });
  // }

  // class Climb extends CommandGroup {
  //   public Climb() {
  //     addSequential(new ConditionalCommand(new CommandClimbLevel2()) {

  //       @Override
  //       protected boolean condition() {
  //         return RobotGameStateManager.climbMode == ClimbMode.LEVEL2;
  //       }
  //     });
  //     addSequential(new ConditionalCommand(new CommandClimbLevel3()) {

  //       @Override
  //       protected boolean condition() {
  //         return RobotGameStateManager.climbMode == ClimbMode.LEVEL3;
  //       }
  //     });
  //   }
  // }
}

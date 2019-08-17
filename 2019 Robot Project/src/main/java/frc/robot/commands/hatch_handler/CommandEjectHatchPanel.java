/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hatch_handler;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.StartCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.bumblelib.bumblelib_autonomous.auto_commands.WaitAndThen;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameStateManager;
import frc.robot.commands.controller.CommandTimedControllerRumble;
import frc.robot.commands.controller.CommandTimedControllerRumble.RumbleLength;
import frc.robot.subsystems.HatchHandler.SolenoidState;

public class CommandEjectHatchPanel extends CommandGroup {

  /**
   * Add your docs here.
   */
  public CommandEjectHatchPanel() {
    addSequential(new InstantCommand(() -> {
      Robot.m_elementArm.setEjectingHatchPanel(true);

      if (RobotGameStateManager.currentGameState.robotAction == RobotAction.PRE_PLACEMENT) {
        RobotGameStateManager.currentGameState.robotAction = RobotAction.POST_PLACEMENT;
      }
    }));
    
    addSequential(new ConditionalCommand(new PushThenFoldHolder(), new FoldHolderThenPush()){
    
      @Override
      protected boolean condition() {
        return Robot.m_hatchHandler.timeToFoldHolderAfterPushExtend >= 0;
      }
    });
    
    // rumble joystick
    addParallel(new StartCommand(
        new CommandTimedControllerRumble(OI.driverController, RumbleType.kLeftRumble, RumbleLength.SHORT, 1.0)));

    addSequential(new SetHatchPush(SolenoidState.FOLD));
    
    addSequential(new WaitAndThen(0.5, new InstantCommand(() -> {
      Robot.m_elementArm.setEjectingHatchPanel(false);
    })));
  }

  @Override
  protected void end() {
    Robot.m_elementArm.setEjectingHatchPanel(false);
  }

  @Override
  protected void interrupted() {
    end();
  }

  class PushThenFoldHolder extends CommandGroup {
    public PushThenFoldHolder() {
      addParallel(new WaitCommand(Robot.m_hatchHandler.hatchReleaseTotalTime)); // Command won't finish until this timeout is finished

      addSequential(new SetHatchPush(SolenoidState.EXTEND));
      addSequential(new WaitCommand(Math.abs(Robot.m_hatchHandler.timeToFoldHolderAfterPushExtend)));
      addSequential(new SetHatchHold(SolenoidState.FOLD));
    }
  }

  class FoldHolderThenPush extends CommandGroup {
    public FoldHolderThenPush() {
      addParallel(new WaitCommand(Robot.m_hatchHandler.hatchReleaseTotalTime)); // Command won't finish until this timeout is finished

      addSequential(new SetHatchHold(SolenoidState.FOLD));
      addSequential(new WaitCommand(Math.abs(Robot.m_hatchHandler.timeToFoldHolderAfterPushExtend)));
      addSequential(new SetHatchPush(SolenoidState.EXTEND));
    }
  }
}

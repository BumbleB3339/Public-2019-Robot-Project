/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.operation.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.Robot;
import frc.robot.commands.cargo_handler.CommandReleaseCargo;
import frc.robot.commands.hatch_handler.CommandEjectHatchPanel;
import frc.robot.subsystems.HatchHandler.SolenoidState;

public class CommandReleaseGamePiece extends CommandGroup {
  /**
   * Add your docs here.
   */

  public CommandReleaseGamePiece() {
    addSequential(new ConditionalCommand(new CommandReleaseCargo(), new CommandEjectHatchPanel()){
    
      @Override
      protected boolean condition() {
        // release cargo if hatch holder is closed, and hatch if it's open
        return Robot.m_hatchHandler.getHoldState() == SolenoidState.FOLD;
      }
    });
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.operation.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

public class CommandCancelClimb extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CommandCancelClimb() {
    addSequential(new InstantCommand(Robot.m_drivetrain, () -> Robot.m_drivetrain.stopMotors()));
    addSequential(new InstantCommand(Robot.m_frontClimb, () -> Robot.m_frontClimb.stop()));
    addSequential(new InstantCommand(Robot.m_rearClimb, () -> Robot.m_rearClimb.stop()));
    addSequential(new InstantCommand(Robot.m_intakeArm, () -> Robot.m_intakeArm.setManualMode(true)));
    addSequential(new InstantCommand(Robot.m_intakeRollers, () -> Robot.m_intakeRollers.stop()));
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo_handler;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class CommadReleaseCargoInAbortCollect extends Command {
  public CommadReleaseCargoInAbortCollect() {
    requires(Robot.m_cargoHandler);
    requires(Robot.m_intakeRollers);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_cargoHandler.releaseCargo();
    Robot.m_intakeRollers.releaseCargo();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_cargoHandler.stop();
    Robot.m_intakeRollers.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

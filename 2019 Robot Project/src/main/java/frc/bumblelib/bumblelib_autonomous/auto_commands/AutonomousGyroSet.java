/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.bumblelib_autonomous.auto_commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bumblelib.bumblelib_autonomous.pathing.AutonomousSettings;
import frc.bumblelib.bumblelib_autonomous.pathing.path.Path;
import frc.robot.Robot;
import jaci.pathfinder.Pathfinder;

public class AutonomousGyroSet extends Command {
  private Path initialPath;

  public AutonomousGyroSet(Path initialPath) {
    this.initialPath = initialPath;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    double yawOffset = Pathfinder.boundHalfDegrees(Math.toDegrees(initialPath.getInitialPathPoint().getWaypoint(AutonomousSettings.getAlliance(),
        AutonomousSettings.getSide()).angle) - Robot.m_drivetrain.getGyro().getRawYaw());
    SmartDashboard.putNumber("permanent offset: ", yawOffset);
    Robot.m_drivetrain.getGyro().setOffset(yawOffset);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

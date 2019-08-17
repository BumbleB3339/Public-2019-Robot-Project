/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.util.maneuvers;

import edu.wpi.first.wpilibj.command.Command;
import frc.bumblelib.util.maneuvers.DriveByDistance;
import frc.bumblelib.util.maneuvers.PIDManeuver;
import frc.robot.Robot;

public class DriveByDistanceCalibration extends Command {

  private PIDManeuver driveManeuver = new DriveByDistance();
  public DriveByDistanceCalibration() {
    requires(Robot.m_drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    driveManeuver.initCalibration();;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    driveManeuver.executeCalibration();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    driveManeuver.end();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

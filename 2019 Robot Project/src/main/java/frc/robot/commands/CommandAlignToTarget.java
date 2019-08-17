/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.vision.AlignToTarget;

public class CommandAlignToTarget extends Command {
  protected AlignToTarget alignToTargetManeuver;
  private boolean isCalibration = false;

  public CommandAlignToTarget() {
    this(false);
  }

  public CommandAlignToTarget(boolean isCalibration) {
    requires(Robot.m_drivetrain);
    alignToTargetManeuver = new AlignToTarget();

    this.isCalibration = isCalibration;

    if (isCalibration) {
      alignToTargetManeuver.initSmartDashboardControls();
    }
  }

  public double getYCoordinate() {
    return alignToTargetManeuver.getYCoordinate();
  }

  public boolean isInProximityMode() {
    return alignToTargetManeuver.isInProximityMode();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (isCalibration) {
      alignToTargetManeuver.initCalibration();
    } else {
      alignToTargetManeuver.init(0);
    }
  }

  // TODO: take from the relevent visual target Processor
  @Override
  protected void execute() {
    if (isCalibration) {
      alignToTargetManeuver.executeCalibration();
    } else {
      alignToTargetManeuver.execute();
    }
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return alignToTargetManeuver.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    alignToTargetManeuver.end();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

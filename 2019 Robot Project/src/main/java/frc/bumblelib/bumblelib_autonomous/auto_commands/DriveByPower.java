/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.bumblelib.bumblelib_autonomous.auto_commands;

import static frc.robot.Robot.m_drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain.DrivetrainControlState;
import frc.robot.subsystems.Drivetrain.Gear;

public class DriveByPower extends Command {
  
  private double leftSidePower;
  private double rightSidePower;
  private Gear gear;
  
  public DriveByPower(double leftSidePower, double rightSidePower, Gear gear) {
    requires(m_drivetrain);

    this.leftSidePower = leftSidePower; 
    this.rightSidePower = rightSidePower; 
    this.gear = gear;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.m_drivetrain.setGear(gear);
    m_drivetrain.initDrivetrainControlState(DrivetrainControlState.OPEN_LOOP);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    m_drivetrain.setLeftRightMotorOutputs(leftSidePower, rightSidePower);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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

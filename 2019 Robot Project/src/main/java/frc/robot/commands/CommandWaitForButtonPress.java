/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.command.Command;

public class CommandWaitForButtonPress extends Command {
  private BooleanSupplier booleanSupplier;
  private boolean lastValue = true;

  public CommandWaitForButtonPress(BooleanSupplier booleanSupplier) {
    this.booleanSupplier = booleanSupplier;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // System.out.println("waiting for button");
    boolean currentValue = booleanSupplier.getAsBoolean();
    boolean returnValue = currentValue && !lastValue;
    lastValue = currentValue;

    return returnValue; // on press
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    lastValue = true;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

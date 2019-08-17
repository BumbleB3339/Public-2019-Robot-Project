/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.command.Command;

public class WaitForTrue extends Command {
  private BooleanSupplier booleanSupplier;

  public WaitForTrue(BooleanSupplier booleanSupplier) {
    this.booleanSupplier = booleanSupplier;
  }
  
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return booleanSupplier.getAsBoolean();
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.controller;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.bumblelib.util.hardware.BumbleController;

/**
 * Add your docs here.
 */
public class CommandTimedControllerRumble extends TimedCommand {
  private BumbleController bumbleController;
  private RumbleType type;
  private double value;

  public enum RumbleLength {
    SHORT(0.3),
    LONG(1.0);

    final double length;
    private RumbleLength(double length) {
      this.length = length;
    }

    public double getLength() {
      return length;
    }
  }

  public CommandTimedControllerRumble(BumbleController bumbleController, RumbleType type, 
    RumbleLength rumbleLength, double value) {
    super(rumbleLength.getLength());
    requires(bumbleController.getControllerSubsystem());
    
    this.bumbleController = bumbleController;
    this.type = type;
    this.value = value;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    bumbleController.setRumble(type, value);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Called once after timeout
  @Override
  protected void end() {
    bumbleController.setRumble(type, 0.0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

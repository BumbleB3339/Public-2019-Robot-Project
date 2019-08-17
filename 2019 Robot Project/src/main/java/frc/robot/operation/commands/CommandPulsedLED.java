/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.operation.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class CommandPulsedLED extends Command {
  private PulseType type;
  private double initTimestamp;
  private boolean isTimed;

  public enum PulseType {
    QUICK(0.1), REGULAR(0.2), SLOW(0.4);

    final double length;

    private PulseType(double length) {
      this.length = length;
    }

    public double getLength() {
      return length;
    }
  }

  public CommandPulsedLED(PulseType type, double timeout) {
    super(timeout);
    requires(Robot.m_pixyVision.LEDSubsystem);
    this.type = type;
    this.isTimed = true;
  }

  public CommandPulsedLED(PulseType type) {
    requires(Robot.m_pixyVision.LEDSubsystem);
    this.type = type;
    this.isTimed = false;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    initTimestamp = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (((int)((Timer.getFPGATimestamp() - initTimestamp) / type.getLength()) % 2) == 1) {
      Robot.m_pixyVision.LED.set(true);
    } else {
      Robot.m_pixyVision.LED.set(false);
    }
  }

  @Override
  protected boolean isFinished() {
    if (isTimed) {
      return super.isTimedOut();
    } else {
      return false;
    }
  }

  // Called once after timeout
  @Override
  protected void end() {
    Robot.m_pixyVision.LED.set(false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

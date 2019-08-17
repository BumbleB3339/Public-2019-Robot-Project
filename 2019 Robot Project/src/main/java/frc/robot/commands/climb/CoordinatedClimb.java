/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Robot.CalibrationMode;
import frc.robot.subsystems.Climb;

public abstract class CoordinatedClimb extends Command {

  private final double CLIMB_STEP_DISTANCE = 3;
  private double currentPIDSetpoint = 0;
  private boolean isBothSidesCoordinated = false;
  private boolean isSteppingFinished = false;

  protected static double bothCurrentPIDSetpoint = 0;

  private final double distanceToActivateFinalKp = 5.0;

  private boolean wasExecuteRunOnce = false;

  private Climb myClimbSubsystem;
  private Climb otherClimbSubsystem;
  private double setpoint;

  protected boolean disablePID;

  public static final double MAX_POWER_UP = 1.0;
  public static final double MAX_POWER_DOWN = 1.0;

  /**
   * 
   */
  public CoordinatedClimb(Climb myClimbSubsystem, Climb otherClimbSubsystem, double setpoint, boolean isBothSync,
      boolean disablePID) {
    requires(myClimbSubsystem);
    this.myClimbSubsystem = myClimbSubsystem;
    this.otherClimbSubsystem = otherClimbSubsystem;
    this.setpoint = setpoint;
    this.isBothSidesCoordinated = isBothSync;
    this.disablePID = disablePID;
  }

  public CoordinatedClimb(Climb myClimbSubsystem, Climb otherClimbSubsystem, double setpoint, boolean isBothSync) {
    requires(myClimbSubsystem);
    this.myClimbSubsystem = myClimbSubsystem;
    this.otherClimbSubsystem = otherClimbSubsystem;
    this.setpoint = setpoint;
    this.isBothSidesCoordinated = isBothSync;
    this.disablePID = false;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    myClimbSubsystem.setFinalSetpoint(setpoint);
    myClimbSubsystem.setMaxPowers(MAX_POWER_UP, MAX_POWER_DOWN);
    isSteppingFinished = false;
    wasExecuteRunOnce = false;
    if (setpoint > myClimbSubsystem.getCurrentHeight()) {
      myClimbSubsystem.setRobotWeightBasePower();
    } else {
      myClimbSubsystem.setAirBasePower();
    }
    myClimbSubsystem.setRouteKp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (!wasExecuteRunOnce) {
      if (isBothSidesCoordinated) {
        bothCurrentPIDSetpoint = (myClimbSubsystem.getCurrentHeight() + otherClimbSubsystem.getCurrentHeight()) / 2.0;
      } else {
        currentPIDSetpoint = myClimbSubsystem.getCurrentHeight();
      }
    }
    if (isBothSidesCoordinated) { // front and rear coordinated climb
      myClimbSubsystem.setSetpoint(bothCurrentPIDSetpoint);
      isSteppingFinished = bothCurrentPIDSetpoint == myClimbSubsystem.getFinalSetpoint();
      if (Math.abs(bothCurrentPIDSetpoint - myClimbSubsystem.getFinalSetpoint()) <= distanceToActivateFinalKp) {
        myClimbSubsystem.setFinalKp();
      }
      if (myClimbSubsystem.isOnTarget() && otherClimbSubsystem.isOnTarget() && !isSteppingFinished
          && myClimbSubsystem == Robot.m_frontClimb) {
        bothCurrentPIDSetpoint += CLIMB_STEP_DISTANCE
            * Math.signum(myClimbSubsystem.getFinalSetpoint() - bothCurrentPIDSetpoint);
        if (Math.abs(bothCurrentPIDSetpoint - myClimbSubsystem.getFinalSetpoint()) <= CLIMB_STEP_DISTANCE) {
          bothCurrentPIDSetpoint = myClimbSubsystem.getFinalSetpoint();
        }
      }
    } else { // no coordination between front and rear climb
      myClimbSubsystem.setSetpoint(currentPIDSetpoint);
      isSteppingFinished = currentPIDSetpoint == myClimbSubsystem.getFinalSetpoint();

      if (Math.abs(currentPIDSetpoint - myClimbSubsystem.getFinalSetpoint()) <= distanceToActivateFinalKp) {
        myClimbSubsystem.setFinalKp();
      }

      if (myClimbSubsystem.isOnTarget() && !isSteppingFinished) {
        currentPIDSetpoint += CLIMB_STEP_DISTANCE
            * Math.signum(myClimbSubsystem.getFinalSetpoint() - currentPIDSetpoint);
        if (Math.abs(currentPIDSetpoint - myClimbSubsystem.getFinalSetpoint()) <= CLIMB_STEP_DISTANCE) {
          currentPIDSetpoint = myClimbSubsystem.getFinalSetpoint();
        }
      }
    }
    wasExecuteRunOnce = true;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Robot.calibrationMode == CalibrationMode.CLIMB) {
      return false;
    }
    return isSteppingFinished && myClimbSubsystem.isOnTarget()
        && (!isBothSidesCoordinated || otherClimbSubsystem.isOnTarget());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if (disablePID) {
      myClimbSubsystem.stop();
    }
    if (Robot.calibrationMode == CalibrationMode.CLIMB) {
      Robot.m_frontClimb.setFinalSetpoint(-1.0);
      Robot.m_rearClimb.setFinalSetpoint(-1.0);
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

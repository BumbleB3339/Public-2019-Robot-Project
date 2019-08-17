/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotGameState.RobotSystemState;
import frc.robot.RobotGameStateManager;

public class SetCurrentRobotSystemState extends InstantCommand {

  private RobotSystemState robotSystemState;
  private boolean waitForOnTarget;

  public SetCurrentRobotSystemState(RobotSystemState robotSystemState, boolean waitForOnTarget) {
    requires(Robot.m_lift);
    requires(Robot.m_liftArm);
    requires(Robot.m_elementArm);
    this.robotSystemState = robotSystemState;
    this.waitForOnTarget = waitForOnTarget;
  }

  @Override
  protected void initialize() {
    RobotGameStateManager.currentGameState.robotSystemState = this.robotSystemState; 
  }

  @Override
  protected boolean isFinished() {
    if (waitForOnTarget) {
      return RobotGameStateManager.isRobotSystemStateOnTarget();
    } else {
      return true;
    }
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo_handler;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameStateManager;

public class CommandReleaseCargo extends Command {

  public CommandReleaseCargo() {
    requires(Robot.m_cargoHandler);
    setTimeout(Robot.m_cargoHandler.REALEASE_TIME);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (RobotGameStateManager.currentGameState.robotAction == RobotAction.PRE_PLACEMENT) {
      RobotGameStateManager.currentGameState.robotAction = RobotAction.POST_PLACEMENT;
    }
    Robot.m_hatchHandler.foldPushSolenoid(); // TEMP BACKUP
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.m_cargoHandler.releaseCargo();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_cargoHandler.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.cargo_handler;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.ControlledGamePieceDetector;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.ControlledGamePieceDetector.ControlledGamePiece;
import frc.robot.subsystems.HatchHandler.SolenoidState;

public class CommandHoldCargoIn extends Command {
  public CommandHoldCargoIn() {
    requires(Robot.m_cargoHandler);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (OI.operatorController.getAButton()) {
      Robot.m_cargoHandler.insertCargo();
    } else if (ControlledGamePieceDetector.getControlledGamePiece() == ControlledGamePiece.CARGO && Robot.m_hatchHandler.getHoldState() != SolenoidState.EXTEND) {
      if (ControlledGamePieceDetector.isHalfControllingCargo()) {
        Robot.m_cargoHandler.strongHoldCargo();
      } else {
        Robot.m_cargoHandler.holdCargo();
      }
      
    } else {
      Robot.m_cargoHandler.stop();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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

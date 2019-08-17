/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hatch_handler;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.HatchHandler.SolenoidState;

public class SetHatchHold extends InstantCommand {
  

  private SolenoidState state;

  public SetHatchHold(SolenoidState state) {
    this.state = state;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    switch (state){
      case FOLD:
        Robot.m_hatchHandler.foldHoldSolenoid();
        break;
      
      case EXTEND:
        Robot.m_hatchHandler.extendHoldSolenoid();
        break;
    }
  }
}

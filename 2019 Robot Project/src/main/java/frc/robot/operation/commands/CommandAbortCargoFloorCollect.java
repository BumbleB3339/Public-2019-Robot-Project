/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.operation.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameState.RobotSystemState;
import frc.robot.RobotGameStateManager;
import frc.robot.commands.SetCurrentRobotSystemState;
import frc.robot.commands.cargo_handler.CommadReleaseCargoInAbortCollect;
import frc.robot.commands.intake_arm.SetIntakeState;
import frc.robot.operation.commands.CommandPulsedLED.PulseType;
import frc.robot.subsystems.IntakeArm.IntakeArmState;

public class CommandAbortCargoFloorCollect extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CommandAbortCargoFloorCollect() {
    addParallel(new CommandPulsedLED(PulseType.QUICK));
    addParallel(new InstantCommand(() -> {
      RobotGameStateManager.currentGameState.robotAction = RobotAction.ABORT_CARGO_FLOOR_COLLECT;
    }));
    addParallel(new CommadReleaseCargoInAbortCollect());
    addParallel(new SetIntakeState(IntakeArmState.ABORT_CARGO_FLOOR_COLLECT, false));
    addSequential(new WaitCommand(0.5));
    addParallel(new SetCurrentRobotSystemState(RobotSystemState.ABORT_CARGO_FLOOR_CARGO, false));
  }
}

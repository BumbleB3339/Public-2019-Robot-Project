/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.operation.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.StartCommand;
import frc.bumblelib.bumblelib_autonomous.auto_commands.Wait;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotGameState.GamePiece;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameStateManager;
import frc.robot.commands.CommandWaitForButtonPress;
import frc.robot.commands.cargo_handler.CommandCollectCargo;
import frc.robot.commands.cargo_handler.StopCargoHandler;
import frc.robot.commands.controller.CommandTimedControllerRumble;
import frc.robot.commands.controller.CommandTimedControllerRumble.RumbleLength;
import frc.robot.commands.hatch_handler.SetHatchHold;
import frc.robot.subsystems.HatchHandler.SolenoidState;

public class CommandFeederCollect extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CommandFeederCollect(boolean forceHatch) {
    requires(Robot.m_lift);
    requires(Robot.m_liftArm);
    requires(Robot.m_elementArm);
    // Set robot action to feeder collect
    // updateCurrentRobotState
    // if game piece is cargo activate cargo handler

    addSequential(new InstantCommand(() -> {
      if (forceHatch) {
        RobotGameStateManager.nextGameState.gamePiece = GamePiece.HATCH_PANEL;
      }
      RobotGameStateManager.nextGameState.robotAction = RobotAction.FEEDER_COLLECT;
      RobotGameStateManager.updateCurrentConfiguration();
    }));

    addParallel(new SetHatchHold(SolenoidState.FOLD));

    addSequential(new ConditionalCommand(new FeederCollectCargo()) {

      @Override
      protected boolean condition() {
        return RobotGameStateManager.currentGameState.gamePiece == GamePiece.CARGO;
      }
    });

    // If current game piece is hatch panel then it ends with a hatch collection
    // motion.
    addSequential(new ConditionalCommand(new FeederCollectHatch()) {

      @Override
      protected boolean condition() {
        return RobotGameStateManager.currentGameState.gamePiece == GamePiece.HATCH_PANEL;
      }
    });
  }

  class FeederCollectCargo extends CommandGroup {
    public FeederCollectCargo() {
      addParallel(new CommandCollectCargo());
      addSequential(new CommandWaitForButtonPress(() -> OI.driverController.getBumper(Hand.kRight)));

      // call with startCommand to prevent the rumble from blocking the finish of the
      // CommandGroup
      addParallel(new StartCommand(
          new CommandTimedControllerRumble(OI.driverController, RumbleType.kLeftRumble, RumbleLength.SHORT, 1.0)));
      addParallel(new StartCommand(
          new CommandTimedControllerRumble(OI.operatorController, RumbleType.kLeftRumble, RumbleLength.SHORT, 1.0)));

      addSequential(new Wait(0.25));
      addSequential(new StopCargoHandler());
    }
  }

  class FeederCollectHatch extends Command {
    private boolean lastButtonState = false;

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
      boolean currentButtonState = OI.driverController.getBumper(Hand.kRight);

      if (currentButtonState && !lastButtonState) { // on press
        Robot.m_hatchHandler.toggleHoldSolenoid();

        switch (Robot.m_hatchHandler.getHoldState()) {
        case FOLD:
          new CommandTimedControllerRumble(OI.driverController, RumbleType.kLeftRumble, RumbleLength.SHORT, 1.0)
              .start();
          break;
        case EXTEND:
          new CommandTimedControllerRumble(OI.operatorController, RumbleType.kLeftRumble, RumbleLength.SHORT, 1.0)
              .start();
          break;
        }
      }

      lastButtonState = currentButtonState;
    }

    @Override
    protected boolean isFinished() {
      return false;
    }

  }
}

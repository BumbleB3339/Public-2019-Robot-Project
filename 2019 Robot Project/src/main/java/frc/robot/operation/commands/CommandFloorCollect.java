/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.operation.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.StartCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.bumblelib.bumblelib_autonomous.auto_commands.Wait;
import frc.bumblelib.bumblelib_autonomous.auto_commands.WaitAndThen;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotGameState.Direction;
import frc.robot.RobotGameState.GamePiece;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameState.RobotSystemState;
import frc.robot.RobotGameStateManager;
import frc.robot.commands.CollectBallWithReverseButton;
import frc.robot.commands.CommandWaitForButtonPress;
import frc.robot.commands.SetCurrentRobotSystemState;
import frc.robot.commands.StopCollectBall;
import frc.robot.commands.WaitForTrue;
import frc.robot.commands.controller.CommandTimedControllerRumble;
import frc.robot.commands.controller.CommandTimedControllerRumble.RumbleLength;
import frc.robot.commands.floor_intake.CollectHatchPanelWithReverseButton;
import frc.robot.commands.floor_intake.DeliverHatchPanel;
import frc.robot.commands.floor_intake.ReleaseHatchPanel;
import frc.robot.commands.floor_intake.StopIntakeRollers;
import frc.robot.commands.hatch_handler.SetHatchHold;
import frc.robot.commands.intake_arm.IntakeArmSetPosition;
import frc.robot.commands.intake_arm.SetIntakeState;
import frc.robot.subsystems.HatchHandler.SolenoidState;
import frc.robot.subsystems.IntakeArm.IntakeArmState;

public class CommandFloorCollect extends CommandGroup {
  public CommandFloorCollect(boolean forceCargo) {
    /**
     * set current robot action to FLOOR_COLLECT. set current direction - FORWARD.
     * set robot state to arm gap (only if intake arm is folded). HATCH: - set
     * intake roller to spin (HATCH) - set intake state to hatch floor collect. -
     * set robot system state to hatch floor collect. - detect hatch in (or by
     * button). - deliver hatch. - disengage CARGO: - set intake roller to spin
     * (cargo) and spin cargo handler rollers - set intake state to cargo floor
     * collect. - set robot system state to cargo floor collect. - detect cargo in
     * (or by button). fold.
     */
    // TODO: prevent game piece change after "detect in" (for either hatch or
    // cargo).
    // TODO: on third click restart command (relevant for hatch)

    addSequential(new InstantCommand(() -> {
      RobotGameStateManager.nextGameState.robotAction = RobotAction.FLOOR_COLLECT;
      RobotGameStateManager.nextGameState.direction = Direction.FORWARD;
      if (forceCargo) {
        RobotGameStateManager.nextGameState.gamePiece = GamePiece.CARGO;
      }

      RobotGameStateManager.updateCurrentConfiguration(); // TODO: Check if this line can be removed
      // if it can be removed, then robot action should update current, and direction
      // should update both current and next
    }));

    addParallel(new SetHatchHold(SolenoidState.FOLD));

    addSequential(
        new ConditionalCommand(new SetCurrentRobotSystemState(RobotSystemState.FRONT_INTAKE_ARM_GAP_LOW, false)) {
          @Override
          protected boolean condition() {
            return Robot.m_intakeArm.isInsideRobot();
          }
        });

    addSequential(new ConditionalCommand(new CommandFloorHatchCollect(), new CommandFloorCargoCollect()) {
      @Override
      protected boolean condition() {
        return RobotGameStateManager.nextGameState.gamePiece == GamePiece.HATCH_PANEL;
      }
    });
  }

  private class CommandFloorHatchCollect extends CommandGroup {
    public CommandFloorHatchCollect() {
      addParallel(new SetIntakeState(IntakeArmState.COLLECT_HATCH));

      addSequential(new CommandWaitForIntakeOut());
      addParallel(new CollectHatchPanelWithReverseButton()); // spin rollers
      addParallel(new SetCurrentRobotSystemState(RobotSystemState.FLOOR_HATCH_START, false));

      addSequential(new CommandWaitForButtonPress(() -> OI.driverController.getStickButton(Hand.kRight)));

      addParallel(new StopIntakeRollers());
      addParallel(new SetIntakeState(IntakeArmState.DELIVER_HATCH_PRESSURE, false));

      // wait for intake to reach 5 degrees from hatch deliver
      addSequential(
          new WaitForTrue(() -> Robot.m_intakeArm.getActualAngle() <= IntakeArmState.DELIVER_HATCH.getAngle() + 5));
      addSequential(new Wait(0.3));

      addSequential(new SetHatchHold(SolenoidState.EXTEND));
      addSequential(new Wait(0.5));
      addParallel(new DeliverHatchPanel());
      addParallel(new SetIntakeState(IntakeArmState.DELIVER_HATCH, false));
      addSequential(new SetCurrentRobotSystemState(RobotSystemState.FLOOR_HATCH_DELIVERY, false));
      addSequential(new WaitCommand(0.1)); // TODO: Adjust according to robot

      addParallel(new SetCurrentRobotSystemState(RobotSystemState.FLOOR_HATCH_END, false));
      addParallel(new ReleaseHatchPanel());
      addParallel(new IntakeArmSetPosition(IntakeArmState.SAFETY_HATCH_EJECT));
      addSequential(new WaitCommand(0.5));
      addSequential(new StopIntakeRollers());

      addSequential(new CommandFoldSystemsAfterHatchFloorCollect());
    }
  }

  class CommandFoldSystemsAfterHatchFloorCollect extends CommandGroup {
    public CommandFoldSystemsAfterHatchFloorCollect() {
      addParallel(new SetIntakeState(IntakeArmState.FOLDED, false));
      addSequential(new WaitForTrue(() -> Robot.m_intakeArm.isApproximatelyFolded()));
      addSequential(new StartCommand(new CommandFoldSystems()));
    }
  }

  class CommandFloorCargoCollect extends CommandGroup {

    public CommandFloorCargoCollect() {

      addParallel(new SetIntakeState(IntakeArmState.COLLECT_CARGO));

      addSequential(new CommandWaitForIntakeOut());

      addParallel(new WaitAndThen(0.2, new CollectBallWithReverseButton())); // spin intake rollers and cargo handler
      addParallel(new SetCurrentRobotSystemState(RobotSystemState.FLOOR_CARGO, false));

      // stop rollers if pressing right stick, or when
      // detecting cargo in
      addSequential(new CommandWaitForButtonPress(() -> false));

      // call with startCommand to prevent the rumble from blocking the finish of the
      // CommandGroup
      addParallel(new StartCommand(
          new CommandTimedControllerRumble(OI.driverController, RumbleType.kLeftRumble, RumbleLength.SHORT, 1.0)));

      addSequential(new Wait(0.25));
      addParallel(new StopCollectBall());

      addSequential((new StartCommand(new CommandFoldSystems())));
    }
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.bumblelib.util.BumbleSupplierTrigger;
import frc.bumblelib.util.hardware.BumbleController;
import frc.robot.RobotGameState.Direction;
import frc.robot.RobotGameState.FieldObjective;
import frc.robot.RobotGameState.GamePiece;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameState.RocketSide;
import frc.robot.commands.CommandAlignToTarget;
import frc.robot.commands.ShiftGear;
import frc.robot.commands.climb.CommandStartClimb;
import frc.robot.commands.hatch_handler.SetHatchHold;
import frc.robot.commands.hatch_handler.SetHatchPush;
import frc.robot.operation.commands.CommandAbortCargoFloorCollect;
import frc.robot.operation.commands.CommandCancelClimb;
import frc.robot.operation.commands.CommandFeederCollect;
import frc.robot.operation.commands.CommandFloorCollect;
import frc.robot.operation.commands.CommandFoldSystems;
import frc.robot.operation.commands.CommandOperatorVerticalPOV;
import frc.robot.operation.commands.CommandOperatorVerticalPOV.POV_VerticalDirection;
import frc.robot.operation.commands.CommandPrepareToClimb;
import frc.robot.operation.commands.CommandPulsedLED;
import frc.robot.operation.commands.CommandPulsedLED.PulseType;
import frc.robot.operation.commands.CommandReleaseGamePiece;
import frc.robot.operation.commands.CommandSetFieldObjective;
import frc.robot.operation.commands.CommandSetGamePiece;
import frc.robot.operation.commands.CommandSetGameStateDirection;
import frc.robot.operation.commands.CommandSetRocketSide;
import frc.robot.operation.commands.CommandSetToPrePlacement;
import frc.robot.operation.commands.CommandToggleClimbMode;
import frc.robot.subsystems.ElementArm.ElementArmState;
import frc.robot.subsystems.HatchHandler.SolenoidState;
import frc.robot.subsystems.IntakeArm.IntakeArmState;
import frc.robot.subsystems.Lift.LiftState;
import frc.robot.subsystems.LiftArm.LiftArmState;
import frc.robot.triggers.TriggerCanceledClimbMode;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

        public static BumbleController driverController = new BumbleController(RobotMap.DRIVER_PORT);
        public static BumbleController operatorController = new BumbleController(RobotMap.OPERATOR_PORT);

        // Operator
        private BumbleSupplierTrigger foldTrigger = new BumbleSupplierTrigger(() -> operatorController.getXButton());
        private BumbleSupplierTrigger feederCollectTrigger = new BumbleSupplierTrigger(() -> operatorController
                        .getYButton()
                        && RobotGameStateManager.currentGameState.robotAction != RobotAction.FLOOR_COLLECT);
        private BumbleSupplierTrigger changeToPrePlacement = new BumbleSupplierTrigger(() -> {
                return operatorController.getBButton()
                                && !(RobotGameStateManager.currentGameState.robotAction == RobotAction.FLOOR_COLLECT
                                                && RobotGameStateManager.currentGameState.gamePiece == GamePiece.HATCH_PANEL);
        });
        private BumbleSupplierTrigger placementLevelUp = new BumbleSupplierTrigger(
                        () -> operatorController.getPOV_Up());
        private BumbleSupplierTrigger placementLevelDown = new BumbleSupplierTrigger(
                        () -> operatorController.getPOV_Down());
        private BumbleSupplierTrigger setGamePieceHatch = new BumbleSupplierTrigger(
                        () -> operatorController.getBumper(Hand.kRight));
        private BumbleSupplierTrigger setGamePieceCargo = new BumbleSupplierTrigger(
                        () -> operatorController.getBumper(Hand.kLeft));
        private BumbleSupplierTrigger setRocketSideLeft = new BumbleSupplierTrigger(
                        () -> operatorController.getPOV_Left());
        private BumbleSupplierTrigger setRocketSideRight = new BumbleSupplierTrigger(
                        () -> operatorController.getPOV_Right());
        private BumbleSupplierTrigger setFieldElementCargoship = new BumbleSupplierTrigger(
                        () -> operatorController.getBackButton());
        private BumbleSupplierTrigger prepareToClimb = new BumbleSupplierTrigger(
                        () -> operatorController.getStartButton());

        // Driver
        private BumbleSupplierTrigger alignToTarget = new BumbleSupplierTrigger(() -> {
                return driverController.getBButton()
                                && RobotGameStateManager.currentGameState.robotAction != RobotAction.CLIMB;
        });

        // double climb AMEN!
        private BumbleSupplierTrigger levitateOnRamp = new BumbleSupplierTrigger(() -> {
                return driverController.getBButton()
                                && RobotGameStateManager.currentGameState.robotAction == RobotAction.CLIMB;
        });
        private BumbleSupplierTrigger foldLeftLeg = new BumbleSupplierTrigger(() -> {
                return RobotGameStateManager.currentGameState.robotAction == RobotAction.CLIMB
                                && operatorController.getPOV_Right();
        });
        private BumbleSupplierTrigger foldRightLeg = new BumbleSupplierTrigger(() -> {
                return RobotGameStateManager.currentGameState.robotAction == RobotAction.CLIMB
                                && operatorController.getPOV_Left();
        });
        // =======
        private BumbleSupplierTrigger toggleClimbMode = new BumbleSupplierTrigger(
                        () -> driverController.getBackButton());
        private BumbleSupplierTrigger cancelClimb = new BumbleSupplierTrigger(() -> driverController.getXButton()
                        && RobotGameStateManager.currentGameState.robotAction == RobotAction.CLIMB);
        private BumbleSupplierTrigger startClimb = new BumbleSupplierTrigger(
                        () -> RobotGameStateManager.currentGameState.robotAction == RobotAction.CLIMB
                                        && driverController.getBumper(Hand.kRight));
        private Trigger shiftGear = new Trigger() {
                @Override
                public boolean get() {
                        return driverController.getBumper(Hand.kLeft);
                }
        };
        private BumbleSupplierTrigger changeDirectionForward = new BumbleSupplierTrigger(
                        () -> driverController.getYButton());
        private BumbleSupplierTrigger changeDirectionBackward = new BumbleSupplierTrigger(
                        () -> driverController.getXButton()
                                        && RobotGameStateManager.currentGameState.robotAction != RobotAction.CLIMB);
        private BumbleSupplierTrigger releaseGamePiece = new BumbleSupplierTrigger(() -> driverController
                        .getBumper(Hand.kRight)
                        && RobotGameStateManager.currentGameState.robotAction != RobotAction.FEEDER_COLLECT
                        && RobotGameStateManager.currentGameState.robotAction != RobotAction.CLIMB
                        && RobotGameStateManager.currentGameState.robotAction != RobotAction.FLOOR_COLLECT
                        && RobotGameStateManager.currentGameState.robotAction != RobotAction.ABORT_CARGO_FLOOR_COLLECT);
        private BumbleSupplierTrigger floorCollectTrigger = new BumbleSupplierTrigger(() ->
        // Make sure we don't repeat the CommandFloorCollect command (we press the
        // button twice for hatch).
        driverController.getStickButton(Hand.kRight) && !RobotGameStateManager.isAnySystemManual()
                        && Robot.m_intakeArm.getCurrentIntakeArmState() != IntakeArmState.MANUAL
                        && RobotGameStateManager.currentGameState.robotAction != RobotAction.FLOOR_COLLECT);
        private BumbleSupplierTrigger floorCollectAbortTrigger = new BumbleSupplierTrigger(() -> (driverController
                        .getBumper(Hand.kRight) || (CommandFoldSystems.isRunning && operatorController.getAButton()))
                        && RobotGameStateManager.currentGameState.robotAction == RobotAction.FLOOR_COLLECT);

        // Manual triggers
        private BumbleSupplierTrigger setLiftArmManual = new BumbleSupplierTrigger(
                        () -> operatorController.getStickButton(Hand.kLeft));
        private BumbleSupplierTrigger setLiftManual = new BumbleSupplierTrigger(
                        () -> operatorController.getTriggerAxis(Hand.kLeft) > 0.7
                                        && operatorController.getTriggerAxis(Hand.kRight) > 0.7);
        private BumbleSupplierTrigger setElementArmManual = new BumbleSupplierTrigger(
                        () -> operatorController.getStickButton(Hand.kRight));
        private BumbleSupplierTrigger setIntakeArmManual = new BumbleSupplierTrigger(
                        () -> driverController.getStartButton());

        // Semi automatic
        private BumbleSupplierTrigger setLiftArmSemiAutomatic = new BumbleSupplierTrigger(
                        () -> Math.abs(operatorController.getY(Hand.kLeft)) > 0.2
                                        && Robot.m_liftArm.getCurrentLiftArmState() != LiftArmState.MANUAL);
        private BumbleSupplierTrigger setElemntArmSemiAutomatic = new BumbleSupplierTrigger(
                        () -> Math.abs(operatorController.getY(Hand.kRight)) > 0.2
                                        && Robot.m_elementArm.getCurrentElementArmState() != ElementArmState.MANUAL);
        private BumbleSupplierTrigger setLiftSemiAutomatic = new BumbleSupplierTrigger(
                        () -> (operatorController.getTriggerAxis(Hand.kRight) > 0.2
                                        || operatorController.getTriggerAxis(Hand.kLeft) > 0.2)
                                        && Robot.m_lift.getCurrentLiftState() != LiftState.MANUAL);

        // Autonomous Triggers.
        public BumbleSupplierTrigger runSemiAutoSequenceNext = new BumbleSupplierTrigger(
                        () -> driverController.getTriggerAxis(Hand.kLeft) > 0.6
                                        && driverController.getTriggerAxis(Hand.kRight) > 0.6 && Robot.isAuto);

        public BumbleSupplierTrigger waitForDriverReleaseTrigger = new BumbleSupplierTrigger(
                        () -> driverController.getBumper(Hand.kRight)); // TODO: edit

        public BumbleSupplierTrigger autoAlignToTargetBreakTrigger = new BumbleSupplierTrigger(
                        () -> driverController.getBumper(Hand.kRight)); // TODO: edit

        public OI() {
                shiftGear.whenActive(new ShiftGear());

                // new operation
                setGamePieceHatch.whenActive(new CommandSetGamePiece(GamePiece.HATCH_PANEL));
                setGamePieceCargo.whenActive(new CommandSetGamePiece(GamePiece.CARGO));

                setRocketSideLeft.whenActive(new CommandSetRocketSide(RocketSide.LEFT));
                setRocketSideRight.whenActive(new CommandSetRocketSide(RocketSide.RIGHT));

                setFieldElementCargoship.whenActive(new CommandSetFieldObjective(FieldObjective.CARGOSHIP));

                changeDirectionForward.whenActive(new CommandSetGameStateDirection(Direction.FORWARD));
                changeDirectionBackward.whenActive(new CommandSetGameStateDirection(Direction.BACKWARD));

                placementLevelUp.whenActive(new CommandOperatorVerticalPOV(POV_VerticalDirection.UP));
                placementLevelDown.whenActive(new CommandOperatorVerticalPOV(POV_VerticalDirection.DOWN));

                changeToPrePlacement.whenActive(new CommandSetToPrePlacement());

                foldTrigger.whenActive(new CommandFoldSystems());
                feederCollectTrigger.whenActive(new CommandFeederCollect(true));

                floorCollectTrigger.whenActive(new CommandFloorCollect(true));
                floorCollectAbortTrigger.whenActive(new CommandAbortCargoFloorCollect());

                releaseGamePiece.whenActive(new CommandReleaseGamePiece());

                // vision
                alignToTarget.whileActive(new CommandAlignToTarget(true)); // boolean is for calibration

                // climb
                prepareToClimb.whenActive(new CommandPrepareToClimb());
                startClimb.whenActive(new CommandStartClimb());
                toggleClimbMode.whenActive(new CommandToggleClimbMode());
                cancelClimb.whenActive(new CommandCancelClimb());

                // special climb
                // levitateOnRamp.whenActive(new AllClimb(25.0));
                // foldLeftLeg.whenActive(new InstantCommand(() -> {
                // Robot.m_rearClimb.leftPIDController.setSetpoint(InitProfiles.ROBOT_PROFILE.climbParams.foldedHeight);
                // Robot.m_rearClimb.foldLeftLeg = true;
                // }));
                // foldRightLeg.whenActive(new InstantCommand(() -> {
                // Robot.m_rearClimb.rightPIDController.setSetpoint(InitProfiles.ROBOT_PROFILE.climbParams.foldedHeight);
                // Robot.m_rearClimb.foldRightLeg = true;
                // }));
                // -----

                // Manual
                setElementArmManual.whenActive(new InstantCommand(() -> {
                        Robot.m_elementArm.toggleManual();
                }));
                setLiftArmManual.whenActive(new InstantCommand(() -> {
                        Robot.m_liftArm.toggleManual();
                }));
                setIntakeArmManual.whenActive(new InstantCommand(() -> {
                        Robot.m_intakeArm.toggleManual();
                }));
                setLiftManual.whenActive(new InstantCommand(() -> {
                        Robot.m_lift.toggleManual();
                }));

                // Semi automatic

                setLiftArmSemiAutomatic.whenActive(new InstantCommand(() -> {
                        Robot.m_liftArm.setLiftArmState(LiftArmState.SEMI_AUTOMATIC);
                }));
                setElemntArmSemiAutomatic.whenActive(new InstantCommand(() -> {
                        Robot.m_elementArm.setElementArmState(ElementArmState.SEMI_AUTOMATIC);
                }));

                // Autonomous.
                BumbleSupplierTrigger waitForDriverReleaseTrigger = new BumbleSupplierTrigger(
                                () -> driverController.getBumper(Hand.kRight)); // TODO: edit

                BumbleSupplierTrigger autoAlignToTargetBreakTrigger = new BumbleSupplierTrigger(
                                () -> driverController.getBumper(Hand.kRight));

                // runSemiAutoSequenceNext.whenActive(Sequences.semiAutoCloseFarRocket.getNext());

                setLiftSemiAutomatic.whenActive(new InstantCommand(() -> {
                        Robot.m_lift.setLiftState(LiftState.SEMI_AUTOMATIC);
                }));

                // temp
                (new BumbleSupplierTrigger(() -> driverController.getPOV_Up()))
                                .whenActive(new SetHatchHold(SolenoidState.EXTEND));
                (new BumbleSupplierTrigger(() -> driverController.getPOV_Down()))
                                .whenActive(new SetHatchHold(SolenoidState.FOLD));
                (new BumbleSupplierTrigger(() -> driverController.getPOV_Down()))
                                .whenActive(new SetHatchPush(SolenoidState.FOLD));

                TriggerCanceledClimbMode canceledClimbMode = new TriggerCanceledClimbMode();
                canceledClimbMode.whenActive(new CommandPulsedLED(PulseType.SLOW, 0.0));
        }
}

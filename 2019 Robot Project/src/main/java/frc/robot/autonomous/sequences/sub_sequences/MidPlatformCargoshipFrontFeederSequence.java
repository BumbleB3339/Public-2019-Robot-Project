/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.sequences.sub_sequences;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.StartCommand;
import frc.bumblelib.bumblelib_autonomous.auto_commands.DrivePathToDistanceFromEnd;
import frc.bumblelib.bumblelib_autonomous.auto_commands.Wait;
import frc.bumblelib.bumblelib_autonomous.auto_commands.WaitAndThen;
import frc.bumblelib.bumblelib_autonomous.pathing.AutonomousSettings;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.robot.Robot;
import frc.robot.RobotGameState.CargoShipFace;
import frc.robot.RobotGameState.Direction;
import frc.robot.RobotGameState.FieldObjective;
import frc.robot.RobotGameState.GamePiece;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameState.RobotSystemState;
import frc.robot.RobotGameState.RocketSide;
import frc.robot.RobotGameStateManager;
import frc.robot.autonomous.commands.CommandAutoAlignToTarget;
import frc.robot.autonomous.paths.Paths;
import frc.robot.commands.SetCurrentRobotSystemState;
import frc.robot.commands.hatch_handler.CommandEjectHatchPanel;
import frc.robot.operation.commands.CommandSetGamePiece;

public class MidPlatformCargoshipFrontFeederSequence extends CommandGroup {
    /**
     * Add your docs here.
     */
    public MidPlatformCargoshipFrontFeederSequence() {
        addSequential(new CommandSetGamePiece(GamePiece.HATCH_PANEL));
        addSequential(new InstantCommand(() -> {
            RobotGameStateManager.nextGameState.rocketSide = AutonomousSettings.getSide() == Side.LEFT ? RocketSide.LEFT
                    : RocketSide.RIGHT;
            RobotGameStateManager.nextGameState.fieldObjective = FieldObjective.CARGOSHIP;
            RobotGameStateManager.currentGameState.fieldObjective = FieldObjective.CARGOSHIP;
            RobotGameStateManager.nextGameState.cargoShipFace = CargoShipFace.FRONT;
            RobotGameStateManager.currentGameState.cargoShipFace = CargoShipFace.FRONT;
            RobotGameStateManager.currentGameState.robotAction = RobotAction.PRE_PLACEMENT;
        }));

        // Drive to rocket and set physical state to placement.
        addParallel(new WaitAndThen(0.0, new SetCurrentRobotSystemState(RobotSystemState.L1_HATCH, true)));
        addParallel(new WaitAndThen(1.0, new InstantCommand(() -> {
            Robot.m_pixyVision.LED.set(true);
            Robot.m_pixyVision.setEnabled(true);
        })));

        addSequential(new DrivePathToDistanceFromEnd(Paths.midPlatformToFrontShipPlacement, 0.05));

        addSequential(new ConditionalCommand(new CommandAutoAlignToTarget(Side.LEFT),
                new CommandAutoAlignToTarget(Side.RIGHT)) {

            @Override
            protected boolean condition() {
                return AutonomousSettings.getSide() == Side.LEFT;
            }
        });
        addParallel(new StartCommand(new CommandEjectHatchPanel()));

        addSequential(new Wait(0.2));

        addParallel(new WaitAndThen(0.6, new InstantCommand(() -> {
            RobotGameStateManager.currentGameState.direction = Direction.BACKWARD;
            RobotGameStateManager.currentGameState.robotAction = RobotAction.FEEDER_COLLECT;
            RobotGameStateManager.nextGameState.robotAction = RobotAction.FEEDER_COLLECT;
            RobotGameStateManager.currentGameState.robotSystemState = RobotSystemState.FEEDER_HATCH_START;
        })));

        addParallel(new WaitAndThen(0.5,
                new InstantCommand(() -> RobotGameStateManager.nextGameState.direction = Direction.BACKWARD)));
        addParallel(new WaitAndThen(2.0, new InstantCommand(() -> {
            Robot.m_pixyVision.LED.set(true);
            Robot.m_pixyVision.setEnabled(true);
        })));

        // Drive to feeder and set physical state to collection.
        addSequential(new DrivePathToDistanceFromEnd(Paths.frontShipPlacementToFeeder, 0.5));

        // Align to target and collect hatch.
        addSequential(new CommandAutoAlignToTarget());
        addParallel(new InstantCommand(() -> {
            Robot.m_hatchHandler.extendHoldSolenoid();
        }));

        addSequential(new Wait(0.3));
    }
}

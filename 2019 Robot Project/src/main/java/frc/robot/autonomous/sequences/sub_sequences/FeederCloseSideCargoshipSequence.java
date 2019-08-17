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
import frc.bumblelib.bumblelib_autonomous.auto_commands.DriveByPower;
import frc.bumblelib.bumblelib_autonomous.auto_commands.DrivePathToDistanceFromEnd;
import frc.bumblelib.bumblelib_autonomous.auto_commands.Wait;
import frc.bumblelib.bumblelib_autonomous.auto_commands.WaitAndThen;
import frc.bumblelib.bumblelib_autonomous.pathing.AutonomousSettings;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.robot.Robot;
import frc.robot.RobotGameState.CargoShipFace;
import frc.robot.RobotGameState.Direction;
import frc.robot.RobotGameState.FieldObjective;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameState.RocketSide;
import frc.robot.RobotGameStateManager;
import frc.robot.autonomous.commands.CommandAutoAlignToTarget;
import frc.robot.autonomous.paths.Paths;
import frc.robot.commands.WaitForTrue;
import frc.robot.commands.hatch_handler.CommandEjectHatchPanel;
import frc.robot.subsystems.Drivetrain.Gear;

public class FeederCloseSideCargoshipSequence extends CommandGroup {
  /**
   * Add your docs here.
   */
  public FeederCloseSideCargoshipSequence() {
    addSequential(new InstantCommand(() -> {
      RobotGameStateManager.nextGameState.rocketSide = AutonomousSettings.getSide() == Side.LEFT ? RocketSide.LEFT
          : RocketSide.RIGHT;
      RobotGameStateManager.nextGameState.fieldObjective = FieldObjective.CARGOSHIP;
      RobotGameStateManager.currentGameState.fieldObjective = FieldObjective.CARGOSHIP;
      RobotGameStateManager.nextGameState.cargoShipFace = CargoShipFace.SIDE;
      RobotGameStateManager.currentGameState.cargoShipFace = CargoShipFace.SIDE;
      
    }));

    addParallel(new WaitAndThen(0.5, new InstantCommand(() -> {
      RobotGameStateManager.nextGameState.direction = Direction.BACKWARD;
      RobotGameStateManager.currentGameState.direction = Direction.BACKWARD;
    })));

    addParallel(new WaitAndThen(0.6, new InstantCommand(() -> {
      RobotGameStateManager.currentGameState.robotAction = RobotAction.PRE_PLACEMENT;
    })));

    addSequential(new DrivePathToDistanceFromEnd(Paths.feederToCloseShipPlacement, 0.05));

    //addSequential(new RotateToAngleFromEnd(new Rotation(Gear.POWER_GEAR, new SymmetricAngle(90), false), 15));
    addParallel(new ConditionalCommand(new DriveByPower(-0.5, 0.5, Gear.POWER_GEAR), new DriveByPower(0.5, -0.5, Gear.POWER_GEAR)) {
    
      @Override
      protected boolean condition() {
        return AutonomousSettings.getSide() == Side.LEFT;
      }
    });
    addSequential(new WaitForTrue(() -> {
      return Math.abs(Math.abs(Robot.m_drivetrain.getGyro().getYaw()) - 90) < 15;
    }));


    addParallel(
        new ConditionalCommand(new CommandAutoAlignToTarget(Side.LEFT), new CommandAutoAlignToTarget(Side.RIGHT)) {

          @Override
          protected boolean condition() {
            return AutonomousSettings.getSide() == Side.RIGHT;
          }
        });
    addSequential(new Wait(1.2));
    addParallel(new DriveByPower(-0.3, -0.3, Gear.POWER_GEAR));
    addSequential(new Wait(0.2));
    addSequential(new WaitForTrue(() -> Robot.m_drivetrain.isDrivetrainStopped()));
    
    addParallel(new StartCommand(new CommandEjectHatchPanel()));

    addSequential(new InstantCommand(() -> {
      RobotGameStateManager.currentGameState.robotAction = RobotAction.POST_PLACEMENT;
    }));

    addSequential(new Wait(0.2));
  }
}

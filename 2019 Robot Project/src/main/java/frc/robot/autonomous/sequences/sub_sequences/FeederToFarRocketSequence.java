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
import frc.bumblelib.bumblelib_autonomous.auto_commands.DriveByPower;
import frc.bumblelib.bumblelib_autonomous.auto_commands.DriveByTime;
import frc.bumblelib.bumblelib_autonomous.auto_commands.DrivePathToDistanceFromEnd;
import frc.bumblelib.bumblelib_autonomous.auto_commands.Wait;
import frc.bumblelib.bumblelib_autonomous.auto_commands.WaitAndThen;
import frc.bumblelib.bumblelib_autonomous.pathing.AutonomousSettings;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.robot.Robot;
import frc.robot.RobotGameState.Direction;
import frc.robot.RobotGameState.FieldObjective;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameState.RobotSystemState;
import frc.robot.RobotGameState.RocketPlacementHeight;
import frc.robot.RobotGameState.RocketSide;
import frc.robot.RobotGameStateManager;
import frc.robot.autonomous.commands.CommandAutoAlignToTarget;
import frc.robot.autonomous.paths.Paths;
import frc.robot.commands.SetCurrentRobotSystemState;
import frc.robot.commands.WaitForTrue;
import frc.robot.commands.hatch_handler.CommandEjectHatchPanel;
import frc.robot.operation.commands.CommandSetFieldObjective;
import frc.robot.operation.commands.CommandSetGameStateDirection;
import frc.robot.subsystems.Drivetrain.Gear;

public class FeederToFarRocketSequence extends CommandGroup {
  /**
   * Add your docs here.
   */
  public FeederToFarRocketSequence() {
    addSequential(new CommandSetFieldObjective(FieldObjective.ROCKET));
    addSequential(new InstantCommand(() -> {
      RobotGameStateManager.nextGameState.rocketSide = AutonomousSettings.getSide() == Side.LEFT ? RocketSide.LEFT
          : RocketSide.RIGHT;
      RobotGameStateManager.nextGameState.rocketPlacementHeight = RocketPlacementHeight.LEVEL2;
      RobotGameStateManager.currentGameState.rocketPlacementHeight = RocketPlacementHeight.LEVEL2;
    }));

    addParallel(new WaitAndThen(0.5, new InstantCommand(() -> {
      RobotGameStateManager.nextGameState.direction = Direction.FORWARD;
    })));

    addParallel(new WaitAndThen(2.0, new InstantCommand(() -> {
      RobotGameStateManager.nextGameState.direction = Direction.BACKWARD;
    })));

    // Drive to rocket and set physical state to placement.
    addParallel(new WaitAndThen(2.0, new SetCurrentRobotSystemState(RobotSystemState.L2_HATCH_FOLDED, true)));
    addParallel(new WaitAndThen(2.0, new InstantCommand(() -> {
      RobotGameStateManager.currentGameState.robotAction = RobotAction.PRE_PLACEMENT;
    })));

    addParallel(new WaitAndThen(2.2, new InstantCommand(() -> {
      Robot.m_pixyVision.LED.set(true);
      Robot.m_pixyVision.setEnabled(true);
    })));

    addSequential(new DrivePathToDistanceFromEnd(Paths.feederToFarRocket, 0.03));
    // addSequential(new Rotate(new Rotation(Gear.POWER_GEAR, new
    // SymmetricAngle((int) InitProfiles.FIELD_PROFILE.farRocketAngle() - 180),
    // false)));

    // Align to target and place hatch.
    addSequential(new CommandAutoAlignToTarget(true));
    addParallel(new DriveByTime(0.2, 0.25, Gear.POWER_GEAR));
    addSequential(new CommandEjectHatchPanel());
    addSequential(new InstantCommand(() -> {
      RobotGameStateManager.currentGameState.robotAction = RobotAction.POST_PLACEMENT;
    }));
    addSequential(new Wait(0.2));
  
    
    addParallel(new WaitAndThen(0.7, new CommandSetGameStateDirection(Direction.FORWARD)));
    addParallel(new ConditionalCommand(new DriveByPower(0.7, 0.1, Gear.SPEED_GEAR), new DriveByPower(0.1, 0.7, Gear.SPEED_GEAR)){
    
      @Override
      protected boolean condition() {
        return AutonomousSettings.getSide() == Side.LEFT;
      }
    });
    addSequential(new WaitForTrue(() -> Math.abs(Robot.m_drivetrain.getGyro().getYaw() - 180.0) < 5.0));
    addSequential(new DriveByTime(0.1, 0.0, Gear.SPEED_GEAR));
  }
}

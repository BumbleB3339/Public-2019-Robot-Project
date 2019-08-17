/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.bumblelib.bumblelib_autonomous.auto_commands.DriveByTime;
import frc.bumblelib.bumblelib_autonomous.auto_commands.Wait;
import frc.robot.Robot;
import frc.robot.RobotGameState.RobotSystemState;
import frc.robot.commands.SetCurrentRobotSystemState;
import frc.robot.commands.WaitForTrue;
import frc.robot.commands.floor_intake.IntakeRollersDragRobot;
import frc.robot.commands.intake_arm.IntakeArmLiftRobot;
import frc.robot.commands.intake_arm.IntakeArmToPosition;
import frc.robot.profiles.InitProfiles;
import frc.robot.subsystems.Drivetrain.Gear;
import frc.robot.subsystems.IntakeArm.IntakeArmState;

public class CommandClimbLevel2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public static final double DRIVE_ROBOT_POWER = 0.25;

  @Override
  protected void initialize() {
    Robot.m_drivetrain.compressor.setClosedLoopControl(false);
  }

  public CommandClimbLevel2() {
    addParallel(new DriveByTime(0.4, CommandClimbLevel3.DRIVE_ROBOT_POWER, Gear.POWER_GEAR));
    addParallel(new IntakeArmLiftRobot(0.6));
    addParallel(new IntakeRollersDragRobot(0.8));
    addSequential(new Wait(0.8));
    addParallel(new IntakeRollersDragRobot(2));
    addParallel(new DriveByTime(2, CommandClimbLevel2.DRIVE_ROBOT_POWER, Gear.POWER_GEAR));
    addSequential(new RearCoordinatedClimb(19, false));

    addParallel(new IntakeRollersDragRobot(0.8));
    addSequential(new DriveByTime(0.8, CommandClimbLevel2.DRIVE_ROBOT_POWER, Gear.POWER_GEAR));
    
    addParallel(new FoldRobotAfterClimb());
    addParallel(new DriveByTime(0.1, -0.1, Gear.POWER_GEAR));    
    addSequential(new RearCoordinatedClimb(InitProfiles.ROBOT_PROFILE.climbParams.foldedHeight, false));
    addSequential(new DriveByTime(1, CommandClimbLevel2.DRIVE_ROBOT_POWER, Gear.POWER_GEAR));
  }

  @Override
  protected void end() {
    Robot.m_frontClimb.setFinalSetpoint(-1.0);
    Robot.m_rearClimb.setFinalSetpoint(-1.0);
  }

  @Override
  protected void interrupted() {
    end();
  }

  class FoldRobotAfterClimb extends CommandGroup {
    public FoldRobotAfterClimb() {
      addParallel(new IntakeArmToPosition(IntakeArmState.FOLDED));
      addSequential(new WaitForTrue(() -> Robot.m_intakeArm.isApproximatelyFolded()));
      addSequential(new SetCurrentRobotSystemState(RobotSystemState.FOLDED, false));
    }
  }
}

// addParallel(new IntakeArmLiftRobot());
// addParallel(new IntakeRollersDragRobot(3));
// addSequential(new DriveByTime(3, CommandClimbLevel3.DRIVE_ROBOT_POWER,
// Gear.POWER_GEAR));
// addParallel(new IntakeArmToPosition(IntakeArmState.FOLDED));
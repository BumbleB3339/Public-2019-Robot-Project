/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.bumblelib.bumblelib_autonomous.auto_commands.DriveByTime;
import frc.bumblelib.bumblelib_autonomous.auto_commands.WaitAndThen;
import frc.robot.Robot;
import frc.robot.commands.WaitForTrue;
import frc.robot.commands.floor_intake.IntakeRollersDragRobot;
import frc.robot.commands.intake_arm.IntakeArmLiftRobot;
import frc.robot.subsystems.Drivetrain.Gear;

public class CommandClimbLevel3 extends CommandGroup {
  public static final double DRIVE_ROBOT_POWER = 0.25;

  @Override
  protected void initialize() {
    Robot.m_drivetrain.compressor.setClosedLoopControl(false);
  }

  public CommandClimbLevel3() {
    addParallel(new DriveByTime(0.4, CommandClimbLevel3.DRIVE_ROBOT_POWER, Gear.POWER_GEAR));
    addParallel(new WaitAndThen(1.0, new IntakeRollersDragRobot(3339)));
    addParallel(new IntakeArmLiftRobot());
    addParallel(new AllClimb(50));
    addSequential(new WaitForTrue(() -> Robot.m_rearClimb.getCurrentHeight() > 50 - 22));
    addParallel(new RearCoordinatedClimb(50, false));
    
    addSequential(new ClimbLevel3Phase2());
    addSequential(new ClimbLevel3Phase3());
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
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.bumblelib.bumblelib_autonomous.auto_commands.DriveByTime;
import frc.robot.Robot;
import frc.robot.commands.floor_intake.IntakeRollersDragRobot;
import frc.robot.commands.intake_arm.IntakeArmLiftRobot;
import frc.robot.subsystems.Drivetrain.Gear;

public class ClimbLevel3Phase2 extends CommandGroup {

  public ClimbLevel3Phase2() {
    addParallel(new IntakeArmLiftRobot());
    addParallel(new DriveByTime(2, CommandClimbLevel3.DRIVE_ROBOT_POWER, Gear.POWER_GEAR));
    addParallel(new IntakeRollersDragRobot(2));
    addSequential(new FrontCoordinatedClimb(ROBOT_PROFILE.climbParams.foldedHeight, false));
    addParallel(new IntakeRollersDragRobot(1.0));
    addParallel(new DriveByTime(1.0, CommandClimbLevel3.DRIVE_ROBOT_POWER, Gear.POWER_GEAR)); // this must by enough time so that the rear legs touch the platform wall
    addSequential(new InstantCommand(Robot.m_intakeArm));
  }
}

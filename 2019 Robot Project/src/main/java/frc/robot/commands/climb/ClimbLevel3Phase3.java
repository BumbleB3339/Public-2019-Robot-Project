/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climb;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.bumblelib.bumblelib_autonomous.auto_commands.DriveByTime;
import frc.robot.Robot;
import frc.robot.RobotGameState.RobotSystemState;
import frc.robot.commands.SetCurrentRobotSystemState;
import frc.robot.commands.WaitForTrue;
import frc.robot.commands.intake_arm.IntakeArmToPosition;
import frc.robot.subsystems.Drivetrain.Gear;
import frc.robot.subsystems.IntakeArm.IntakeArmState;

public class ClimbLevel3Phase3 extends CommandGroup {

  public ClimbLevel3Phase3() {
    addParallel(new FoldRobotAfterClimb());
    addParallel(new DriveByTime(0.1, -0.1, Gear.POWER_GEAR));
    addSequential(new RearCoordinatedClimb(ROBOT_PROFILE.climbParams.foldedHeight, false));
    addSequential(new DriveByTime(1, CommandClimbLevel3.DRIVE_ROBOT_POWER, Gear.POWER_GEAR));
  }

  class FoldRobotAfterClimb extends CommandGroup {
    public FoldRobotAfterClimb() {
      addParallel(new IntakeArmToPosition(IntakeArmState.FOLDED));
      addSequential(new WaitForTrue(() -> Robot.m_intakeArm.isApproximatelyFolded()));
      addSequential(new SetCurrentRobotSystemState(RobotSystemState.FOLDED, false));
    }
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake_arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.profiles.InitProfiles;
import frc.robot.subsystems.IntakeArm.IntakeArmState;

public class IntakeArmLiftRobot extends Command {

  private boolean isTimeoutForFullSpeed = false;
  private double timeoutForFullSpeed;
  private double beginningTimestamp;

  public IntakeArmLiftRobot() {
    requires(Robot.m_intakeArm);
  }

  public IntakeArmLiftRobot(double timeoutForFullSpeed) {
    requires(Robot.m_intakeArm);
    isTimeoutForFullSpeed = true;
    this.timeoutForFullSpeed = timeoutForFullSpeed;
  }

  @Override
  protected void initialize() {
    Robot.m_intakeArm.setPosition(IntakeArmState.CLIMB);
    beginningTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  protected void execute() {
    // double powerToApply = Robot.m_intakeArm.getActualAngle() < InitProfiles.ROBOT_PROFILE.intakeArmParams.holdRobotAngle
    //     ? InitProfiles.ROBOT_PROFILE.intakeArmParams.liftRobotPower
    //     : InitProfiles.ROBOT_PROFILE.intakeArmParams.supportRobotPower;
    double powerToApply = InitProfiles.ROBOT_PROFILE.intakeArmParams.liftRobotPower;
    if (isTimeoutForFullSpeed && Timer.getFPGATimestamp() - beginningTimestamp < timeoutForFullSpeed) {
      powerToApply = 1.0;
    }

    Robot.m_intakeArm.setPower(powerToApply);
  }

  @Override
  protected void end() {
    Robot.m_intakeArm.stop();
  }

  @Override
  protected void interrupted() {
    end();
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}

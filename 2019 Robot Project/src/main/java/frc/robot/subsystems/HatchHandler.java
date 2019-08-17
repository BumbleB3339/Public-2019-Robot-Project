/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.bumblelib.util.SmartdashboardDebugging;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class HatchHandler extends Subsystem implements SmartdashboardDebugging {

  public enum SolenoidState {
    FOLD, EXTEND;
  }

  public double hatchReleaseTotalTime = 0.6;
  public double timeToFoldHolderAfterPushExtend = ROBOT_PROFILE.hatchHandlerParams.timeToFoldHolderAfterPushExtend;
  private Solenoid pushSolenoid;
  private DoubleSolenoid holdSolenoid;
  private SolenoidState pushState, holdState;

  public HatchHandler() {
    pushSolenoid = new Solenoid(RobotMap.HatchHandlerPorts.HATCH_PUSH_SOLENOID);
    holdSolenoid = new DoubleSolenoid(RobotMap.HatchHandlerPorts.HATCH_HOLD_SOLENOID_FORWARD,
        RobotMap.HatchHandlerPorts.HATCH_HOLD_SOLENOID_REVERSE);

    // TODO: TEMP for testing
    // SmartDashboard.putNumber("TIMEOUT_TO_FOLD_HATCH_PUSH",
    // TIMEOUT_TO_FOLD_HATCH_PUSH);
    // SmartDashboard.putNumber("TIMEOUT_TO_FOLD_HATCH_HOLD",
    // TIMEOUT_TO_FOLD_HATCH_HOLD);

  }

  public void setDefaultSolenoidStates() {
    foldPushSolenoid();
    foldHoldSolenoid();
  }

  @Override
  public void initDefaultCommand() {
  }

  public void extendPushSolenoid() {
    pushSolenoid.set(true);
    pushState = SolenoidState.EXTEND;
  }

  public void foldPushSolenoid() {
    pushSolenoid.set(false);
    pushState = SolenoidState.FOLD;
  }

  public void foldHoldSolenoid() {
    holdSolenoid.set(Value.kReverse);
    holdState = SolenoidState.FOLD;
  }

  public void extendHoldSolenoid() {
    holdSolenoid.set(Value.kForward);
    holdState = SolenoidState.EXTEND;
  }

  public void toggleHoldSolenoid() {
    if (holdState == SolenoidState.FOLD) {
      extendHoldSolenoid();
    } else {
      foldHoldSolenoid();
    }
  }

  /**
   * @return the holdState
   */
  public SolenoidState getHoldState() {
    return holdState;
  }

  /**
   * @return the pushState
   */
  public SolenoidState getPushState() {
    return pushState;
  }

  @Override
  public void sendDebuggingData() {

  }

  @Override
  public void periodic() {
    // TIMEOUT_TO_FOLD_HATCH_HOLD =
    // SmartDashboard.getNumber("TIMEOUT_TO_FOLD_HATCH_HOLD", 0.0);
    // TIMEOUT_TO_FOLD_HATCH_PUSH =
    // SmartDashboard.getNumber("TIMEOUT_TO_FOLD_HATCH_PUSH", 0.0);
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.bumblelib.util.SmartdashboardDebugging;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class IntakeRollers extends Subsystem implements SmartdashboardDebugging {

  private final double COLLECT_HATCH_PANEL_POWER = ROBOT_PROFILE.floorRollerParams.collectHatchPanelPower;
  private final double RELEASE_HATCH_PANEL_POWER = ROBOT_PROFILE.floorRollerParams.releaseHatchPanelPower;
  private final double COLLECT_CARGO_POWER = ROBOT_PROFILE.floorRollerParams.collectCargoPower;
  private final double DELIVER_HATCH_PANEL_POWER = ROBOT_PROFILE.floorRollerParams.deliverHatchPanelPower;
  private final double DRAG_POWER = ROBOT_PROFILE.floorRollerParams.dragPower;

  private final boolean IS_MOTOR_INVERTED = ROBOT_PROFILE.floorRollerParams.isMotorInverted;
  private final NeutralMode DEFAULT_NEUTRAL_MODE = NeutralMode.Coast;

  private WPI_VictorSPX floorIntakeMotor;

  public IntakeRollers() {
    floorIntakeMotor = new WPI_VictorSPX(RobotMap.FloorIntakePorts.MOTOR);
    floorIntakeMotor.setInverted(IS_MOTOR_INVERTED);
    floorIntakeMotor.setNeutralMode(DEFAULT_NEUTRAL_MODE);
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new CommandIntakeRollersReleaseCargoButton());
  }

  public boolean isHatchPanelIn() {
    return false; // TODO: Implement with current detection
  }

  public void stop() {
    floorIntakeMotor.set(0);
  }

  public void collectHatchPanel() {
    floorIntakeMotor.setNeutralMode(NeutralMode.Coast);
    floorIntakeMotor.set(COLLECT_HATCH_PANEL_POWER);
  }

  public void releaseHatchPanel() {
    floorIntakeMotor.setNeutralMode(NeutralMode.Brake);
    floorIntakeMotor.set(RELEASE_HATCH_PANEL_POWER);
  }

  public void deliverHatchPanel() {
    floorIntakeMotor.setNeutralMode(NeutralMode.Brake);
    floorIntakeMotor.set(DELIVER_HATCH_PANEL_POWER);
  }

  public void collectCargo() {
    floorIntakeMotor.setNeutralMode(NeutralMode.Coast);
    floorIntakeMotor.set(COLLECT_CARGO_POWER);
  }

  public void releaseCargo() {
    floorIntakeMotor.setNeutralMode(NeutralMode.Coast);
    floorIntakeMotor.set(-COLLECT_CARGO_POWER);
  }

  public void dragRobot() {
    floorIntakeMotor.setNeutralMode(NeutralMode.Brake);
    floorIntakeMotor.set(DRAG_POWER);
  }

  public void setPower(double power) {
    floorIntakeMotor.setNeutralMode(NeutralMode.Brake);
    floorIntakeMotor.set(power);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Intake Rollers Current", floorIntakeMotor.get) //
    // pdp 4
  }

  @Override
  public void sendDebuggingData() {
  }
}

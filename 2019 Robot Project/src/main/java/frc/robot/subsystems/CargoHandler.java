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
import frc.robot.commands.cargo_handler.CommandHoldCargoIn;

/**
 * Add your docs here.
 */
public class CargoHandler extends Subsystem implements SmartdashboardDebugging {

  public final double REALEASE_TIME = 1.5;
  private final double INSERT_POWER = -1;
  private final double HOLD_POWER = -0.3;
  private final double RELEASE_POWER = 1;
  private final double STRONG_HOLD_POWER = -0.5;
  private final NeutralMode DEFAULT_NEUTRAL_MODE = NeutralMode.Brake;

  private final boolean IS_MOTOR_INVERTED = ROBOT_PROFILE.cargoHandlerParams.isMotorInverted;
  private WPI_VictorSPX cargoHandlerMotor;

  public CargoHandler() {
    cargoHandlerMotor = new WPI_VictorSPX(RobotMap.CargoHandlerPorts.MOTOR);
    cargoHandlerMotor.setInverted(IS_MOTOR_INVERTED);
    cargoHandlerMotor.setNeutralMode(DEFAULT_NEUTRAL_MODE);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CommandHoldCargoIn()); // TODO: Check and enable
  }

  public void strongHoldCargo() {
    cargoHandlerMotor.set(STRONG_HOLD_POWER);
  }

  public void insertCargo() {
    cargoHandlerMotor.set(INSERT_POWER);
  }

  public void releaseCargo() {
    cargoHandlerMotor.set(RELEASE_POWER);
  }

  public void holdCargo() {
    cargoHandlerMotor.set(HOLD_POWER);
  }

  public void stop() {
    cargoHandlerMotor.set(0);
  }

  @Override
  public void sendDebuggingData() {
  }
}

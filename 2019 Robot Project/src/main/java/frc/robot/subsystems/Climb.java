/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.bumblelib.util.BumblePIDController;
import frc.bumblelib.util.SmartdashboardDebugging;
import frc.bumblelib.util.hardware.BumblePotentiometer;

/**
 * Add your docs here.
 */
public abstract class Climb extends Subsystem implements SmartdashboardDebugging {

  public SpeedController leftMotor; // Should be public to access its sensor from another classes.
  public SpeedController rightMotor; // Should be public to access its sensor from another classes.
  public BumblePotentiometer leftPotentiometer;
  public BumblePotentiometer rightPotentiometer;

  public boolean foldLeftLeg = false;
  public boolean foldRightLeg = false;

  public BumblePIDController rightPIDController;
  public BumblePIDController leftPIDController;

  public double MAX_POWER_UP = 1.0; // TODO: This will be a final protected
  public double MAX_POWER_DOWN = 1.0; // TODO: So will this one be
  protected final double TOLERANCE = 3;

  public double routeKp = 1.0;
  public double finalKp = 0.12;
  public double robotWeightBasePower = 0.08; // This should be positive
  public double airBasePower = 0.0; // This should be positive

  private final double APPLY_ONLY_BASEPOWER_TOLERANCE = 1.0;

  private double finalSetpoint = -1.0; // Setpoint of where the climb wants to get, not necessarily its current
                                       // PIDController setpoint

  public double getCurrentHeight() {
    return (leftPotentiometer.get() + rightPotentiometer.get()) / 2.0;
  }

  public boolean isOnTarget() {
    return rightPIDController.onTarget() && leftPIDController.onTarget();
  }

  @Override
  public void periodic() {
    if (finalSetpoint > 0.0) {
      if (Math.abs(rightPotentiometer.get() - (finalSetpoint - 0.5 * APPLY_ONLY_BASEPOWER_TOLERANCE)) < 0.5
          * APPLY_ONLY_BASEPOWER_TOLERANCE && !foldRightLeg) {
        rightPIDController.disable();
        rightMotor.set(robotWeightBasePower);
      } else {
        rightPIDController.enable();
      }

      if (Math.abs(leftPotentiometer.get() - (finalSetpoint - 0.5 * APPLY_ONLY_BASEPOWER_TOLERANCE)) < 0.5
          * APPLY_ONLY_BASEPOWER_TOLERANCE && !foldLeftLeg) {
        leftPIDController.disable();
        leftMotor.set(robotWeightBasePower);
      } else {
        leftPIDController.enable();
      }
    }
  }

  public double getCurrentSetpoint() {
    return rightPIDController.getSetpoint();
  }

  public void setFinalSetpoint(double finalSetpoint) {
    this.finalSetpoint = finalSetpoint;
  }

  public double getFinalSetpoint() {
    return finalSetpoint;
  }

  public void setMaxPowers(double maxPowerUp, double maxPowerDown) {
    rightPIDController.setOutputRange(-maxPowerDown, maxPowerUp);
    leftPIDController.setOutputRange(-maxPowerDown, maxPowerUp);
  }

  public void setSetpoint(double height) {
    rightPIDController.setSetpoint(height);
    leftPIDController.setSetpoint(height);

    rightPIDController.enable();
    leftPIDController.enable();
  }

  public void setRouteKp() {
    rightPIDController.setP(routeKp);
    leftPIDController.setP(routeKp);
  }

  public void setFinalKp() {
    rightPIDController.setP(finalKp);
    leftPIDController.setP(finalKp);
  }

  public void setBothPower(double power) {
    setRightPower(power);
    setLeftPower(power);
  }

  public void setRightPower(double power) {
    rightMotor.set(power);
  }

  public void setLeftPower(double power) {
    leftMotor.set(power);
  }

  public abstract void setRobotWeightBasePower();

  public abstract void setAirBasePower();

  public void stop() {
    rightPIDController.disable();
    leftPIDController.disable();
    setBothPower(0.0);
  }
}

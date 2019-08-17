/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bumblelib.util.BasePowerCalculator;
import frc.bumblelib.util.BumbleTimer;
import frc.bumblelib.util.PIDPreset;
import frc.bumblelib.util.SmartdashboardDebugging;
import frc.bumblelib.util.StableBoolean;
import frc.bumblelib.util.VelocityKalmanFilter;
import frc.bumblelib.util.hardware.BasePowerWPI_VictorSPX;
import frc.bumblelib.util.hardware.BumblePotentiometer;
import frc.robot.ControlledGamePieceDetector;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotGameStateManager;
import frc.robot.RobotMap;
import frc.robot.commands.controller.CommandTimedControllerRumble;
import frc.robot.commands.controller.CommandTimedControllerRumble.RumbleLength;
import frc.robot.subsystems.LiftArm.LiftArmState;
import jaci.pathfinder.Pathfinder;

/**
 * Add your docs here.
 */
public class ElementArm extends Subsystem implements SmartdashboardDebugging {

  public BumblePotentiometer elementArmPotentiometer;
  public BasePowerWPI_VictorSPX elementArmMotor; // Should be public to access its sensor from another classes.
  public PIDController pidController;
  private double gravityCompensationPower = 0;
  private VelocityKalmanFilter velocityKalmanFilter = new VelocityKalmanFilter(25);

  private double currentAngleSetpoint = -3339;

  private final int ON_TARGET_TRUE_THRESHOLD = 10;
  private final int ON_TARGET_FALSE_THRESHOLD = 5;
  private StableBoolean onTarget = new StableBoolean(ON_TARGET_TRUE_THRESHOLD, ON_TARGET_FALSE_THRESHOLD);

  private final ElementArmState DEFAULT_ELEMENT_ARM_STATE = ElementArmState.MANUAL;
  private final boolean IS_MOTOR_INVERTED = ROBOT_PROFILE.elementArmParams.isMotorInverted;
  private final NeutralMode DEFAULT_NEUTRAL_MODE = NeutralMode.Brake;

  private final double MANUAL_POWER_FORWARD = ROBOT_PROFILE.elementArmParams.manualPowerForward;
  private final double MANUAL_POWER_BACKWARD = ROBOT_PROFILE.elementArmParams.manualPowerBackward;

  // On target params
  public final double POSITION_TOLERANCE_ANGLE = 3;
  public double VELOCITY_TOLERANCE = 100000000;
  public double frictionOvercomePower = ROBOT_PROFILE.elementArmParams.frictionOvercomePower;
  public double GRAVITY_F_NONE = ROBOT_PROFILE.elementArmParams.gravityF_none;
  public double GRAVITY_F_CARGO = ROBOT_PROFILE.elementArmParams.gravityF_cargo;
  public double GRAVITY_F_HATCH_PANEL = ROBOT_PROFILE.elementArmParams.gravityF_hatchPanel;

  public boolean isGravityFCalibrationMode = false;
  public double calibrationGravityF = 0.0;

  public final double RAMP_RATE = 0.1;

  private final PIDPreset pidPreset_none = ROBOT_PROFILE.elementArmParams.pidPreset_none;
  private final PIDPreset pidPreset_cargo = ROBOT_PROFILE.elementArmParams.pidPreset_cargo;
  private final PIDPreset pidPreset_hatch = ROBOT_PROFILE.elementArmParams.pidPreset_hatch;

  private final double MAX_ANGLE = 135;
  private final double MAX_OUTPUT = ROBOT_PROFILE.elementArmParams.outputRange;
  private final double MAX_OUTPUT_ON_TARGET = 0;

  private boolean isEjectingHatchPanel = false;
  private final double GRAVITY_F_EJECTING = ROBOT_PROFILE.elementArmParams.gravityF_Ejecting;

  private ElementArmState currentElementArmState = DEFAULT_ELEMENT_ARM_STATE;

  public enum ElementArmState {
    MANUAL(), //
    CALIBRATION(), //
    SEMI_AUTOMATIC(), // Always keep angle relative to LiftArm (used when LiftArm is in manual mode)
    FOLLOWING_CURRENT_STATE();

    public String toDashboardString() {
      switch (this) {
      case MANUAL:
        return "Manual";
      case CALIBRATION:
        return "Calibration";
      case SEMI_AUTOMATIC:
        return "Semi Automatic";
      default:
        return "Automatic";
      }
    }
  }

  public ElementArm() {
    elementArmMotor = new BasePowerWPI_VictorSPX(RobotMap.ElementArmPorts.MOTOR, frictionOvercomePower);
    elementArmMotor.setInverted(IS_MOTOR_INVERTED);
    elementArmMotor.setNeutralMode(DEFAULT_NEUTRAL_MODE);
    elementArmMotor.configVoltageCompSaturation(11);
    elementArmMotor.enableVoltageCompensation(true);
    elementArmMotor.configOpenloopRamp(RAMP_RATE);

    elementArmPotentiometer = new BumblePotentiometer(RobotMap.ElementArmPorts.POTENTIOMETER, -90,
        ROBOT_PROFILE.elementArmParams.voltageInAngleMinus90, 90, ROBOT_PROFILE.elementArmParams.voltageInAngle90,
        -MAX_ANGLE - 10, MAX_ANGLE + 10);

    pidController = new PIDController(pidPreset_none.getKp(), pidPreset_none.getKi(), pidPreset_none.getKd(),
        new ActualAnglePIDSource(), elementArmMotor);
    pidController.disable();
    pidController.setAbsoluteTolerance(POSITION_TOLERANCE_ANGLE);
    setElementArmState(DEFAULT_ELEMENT_ARM_STATE);
  }

  public double getCurrentSetpoint() {
    return pidController.getSetpoint();
  }

  @Override
  public void initDefaultCommand() {
    // nothing
  }

  public void setPower(double power) {
    elementArmMotor.set(power);
  }

  public void stop() {
    elementArmMotor.set(0);
  }

  public void manualElementArmForward() {
    elementArmMotor.set(MANUAL_POWER_FORWARD);
  }

  public void manualElementArmBackward() {
    elementArmMotor.set(MANUAL_POWER_BACKWARD);
  }

  public void applyPIDPreset(PIDPreset pidPreset) {
    pidController.setPID(pidPreset.getKp(), pidPreset.getKi(), pidPreset.getKd());
  }

  public void applyPIDPreset(double kP, double kI, double kD) {
    pidController.setPID(kP, kI, kD);
  }

  public void setPIDSetpoint(double angleSetpoint) {
    if (getCurrentElementArmState() == ElementArmState.MANUAL) {
      return;
    }

    if (currentAngleSetpoint != angleSetpoint) {
      onTarget.forceValue(false);
    }
    this.currentAngleSetpoint = angleSetpoint;
    pidController.setSetpoint(angleSetpoint);
  }

  public void setElementArmState(ElementArmState elementArmState) {
    this.currentElementArmState = elementArmState;
    if (elementArmState == ElementArmState.FOLLOWING_CURRENT_STATE) {
      pidController.enable();
    } else {
      pidController.disable();
    }
    if (elementArmState == ElementArmState.SEMI_AUTOMATIC) {
      RobotGameStateManager.setRobotSystemStateToSemiAuto();
    }
  }

  public double getActualAngle() {
    if (getAbsoluteAngle() < 0) {
      return Pathfinder
          .boundHalfDegrees(this.elementArmPotentiometer.get() - ROBOT_PROFILE.elementArmParams.angleFreedom);
    }
    return this.elementArmPotentiometer.get();
  }

  public double getAbsoluteAngle() {
    return Robot.m_liftArm.getActualAngle() + this.elementArmPotentiometer.get();
  }

  private final int ITERATIONS_TO_DETERMINE_DEFECTIVE = 15;
  private int defectiveCounter = 0;
  private boolean lastIsPotentiometerDefective = false;
  private boolean isPotentiometerDefective = false;

  private void checkPotentiometer() {
    defectiveCounter = elementArmPotentiometer.isDefective() ? defectiveCounter + 1 : 0;
    boolean isDefective = defectiveCounter >= ITERATIONS_TO_DETERMINE_DEFECTIVE;
    isPotentiometerDefective = isDefective;
    if (isDefective && !lastIsPotentiometerDefective) {
      setElementArmState(ElementArmState.MANUAL);
      DriverStation.reportError("ElementArm potentiometer out of range, ElementArm is set to MANUAL", false);
    }
    lastIsPotentiometerDefective = isDefective;
  }

  public boolean isPotentiometerDefective() {
    return isPotentiometerDefective;
  }

  // Velocity related
  public double getCurrentVelocity() {
    return velocityKalmanFilter.getSmoothValue();
  }

  private void updateCurrentVelocity() {
    velocityKalmanFilter.addValue(getActualAngle());
  }
  // =========

  private void updateOnTarget() {
    onTarget.update(pidController.onTarget()); // && Math.abs(getCurrentVelocity()) < VELOCITY_TOLERANCE)
  }

  public boolean isOnTarget() {
    return getCurrentElementArmState() == ElementArmState.MANUAL || onTarget.get();
  }

  private double lastUpdatedMaxOutput = -3339;

  public void updatePidOutputRange() {
    // works only during FOLLOWING_CURRENT_STATE
    if (getCurrentElementArmState() != ElementArmState.FOLLOWING_CURRENT_STATE) {
      return;
    }

    double selectedMaxOutput;
    if (isOnTarget()) {
      selectedMaxOutput = 0.0;
    } else {
      selectedMaxOutput = 1.0;
    }

    if (selectedMaxOutput != lastUpdatedMaxOutput) {
      pidController.setOutputRange(-selectedMaxOutput, selectedMaxOutput);
      lastUpdatedMaxOutput = selectedMaxOutput;
    }
  }

  public void setWithBasePower(double output) {
    setPower(BasePowerCalculator.calculateModifiedOutput(output, -gravityCompensationPower, frictionOvercomePower));
  }

  /**
   * This methods periodically calculates the base power to the arm PID loop and
   * applies it. It is derived from
   * https://drive.google.com/file/d/1FUZyMC-_ZrIn2FgOAd0_Q0TUYOHEyPpA/view?usp=sharing
   */
  private void updateGravityCompensationPower() {
    double gravityF;
    if (isGravityFCalibrationMode) {
      gravityF = calibrationGravityF;
    } else {
      gravityF = getGravityF();
    }
    gravityCompensationPower = gravityF * Math.sin(Math.toRadians(getAbsoluteAngle()));
    elementArmMotor.setGravityCompensationPower(-gravityCompensationPower);
  }

  private double getGravityF() {
    if (isEjectingHatchPanel) {
      return GRAVITY_F_EJECTING;
    } else {
      switch (ControlledGamePieceDetector.getControlledGamePiece()) {
      case CARGO:
        return GRAVITY_F_CARGO;
      case HATCH_PANEL:
        return GRAVITY_F_HATCH_PANEL;
      case NONE:
      default:
        return GRAVITY_F_NONE;
      }
    }
  }

  /**
   * @param isEjectingHatchPanel the isEjectingHatchPanel to set
   */
  public void setEjectingHatchPanel(boolean isEjectingHatchPanel) {
    this.isEjectingHatchPanel = isEjectingHatchPanel;
  }

  private static PIDPreset lastUpdatedPidPreset = null;

  private void updatePidPresetAccordingToGamePiece() {
    if (getCurrentElementArmState() == ElementArmState.CALIBRATION) {
      return;
    }

    PIDPreset selectedPidPreset;
    switch (ControlledGamePieceDetector.getControlledGamePiece()) {
    case CARGO:
      selectedPidPreset = pidPreset_cargo;
      break;
    case HATCH_PANEL:
      selectedPidPreset = pidPreset_hatch;
      break;
    case NONE:
    default:
      selectedPidPreset = pidPreset_none;
      break;
    }

    if (selectedPidPreset != lastUpdatedPidPreset) {
      applyPIDPreset(selectedPidPreset);
      lastUpdatedPidPreset = selectedPidPreset;
    }
  }

  public ElementArmState getCurrentElementArmState() {
    return currentElementArmState;
  }

  private LiftArmState lastLiftArmState;

  private void toggleSemiAutomatic() {
    if (getCurrentElementArmState() == ElementArmState.MANUAL) {
      return;
    }

    LiftArmState currentLiftArmState = Robot.m_liftArm.getCurrentLiftArmState();

    if (getCurrentElementArmState() != ElementArmState.SEMI_AUTOMATIC && currentLiftArmState == LiftArmState.MANUAL) {
      // change to semi-automatic
      setElementArmState(ElementArmState.SEMI_AUTOMATIC);
    } else if (getCurrentElementArmState() == ElementArmState.SEMI_AUTOMATIC && lastLiftArmState == LiftArmState.MANUAL
        && currentLiftArmState != LiftArmState.MANUAL) {
      // change back to normal
      setElementArmState(ElementArmState.SEMI_AUTOMATIC);
    }

    lastLiftArmState = currentLiftArmState;
  }

  public void updatePidMaxOutput() {
    if (isEjectingHatchPanel) {
      pidController.setOutputRange(0.0, 0.0);
    } else if (isOnTarget()) {
      // limit to small values when on target
      pidController.setOutputRange(-MAX_OUTPUT_ON_TARGET, MAX_OUTPUT_ON_TARGET);
    } else {
      pidController.setOutputRange(-MAX_OUTPUT, MAX_OUTPUT);
    }
  }

  private void manualControl() {
    if (getCurrentElementArmState() != ElementArmState.MANUAL || !Robot.isOperationActive) {
      return;
    }

    if (Math.abs(OI.operatorController.getY(Hand.kRight)) > 0.2) {
      elementArmMotor.set(-(OI.operatorController.getY(Hand.kRight)));
    } else {
      elementArmMotor.set(0.0);
    }
  }

  private void semiAutomaticControl() {
    if (getCurrentElementArmState() != ElementArmState.SEMI_AUTOMATIC || !Robot.isOperationActive) {
      return;
    }

    if (Math.abs(OI.operatorController.getY(Hand.kRight)) > 0.2) {
      setWithBasePower(-OI.operatorController.getY(Hand.kRight));
    } else {
      setWithBasePower(0.0);
    }
  }

  private CommandTimedControllerRumble manualEnterRumbleCommand;
  private CommandTimedControllerRumble manualFinishRumbleCommand;

  public void toggleManual() {
    if (manualEnterRumbleCommand == null) {
      manualEnterRumbleCommand = new CommandTimedControllerRumble(OI.operatorController, RumbleType.kRightRumble,
          RumbleLength.LONG, 1.0);
    }
    if (manualFinishRumbleCommand == null) {
      manualFinishRumbleCommand = new CommandTimedControllerRumble(OI.operatorController, RumbleType.kRightRumble,
          RumbleLength.SHORT, 1.0);
    }

    if (getCurrentElementArmState() == ElementArmState.MANUAL) {
      setElementArmState(ElementArmState.SEMI_AUTOMATIC);
      manualFinishRumbleCommand.start();
    } else {
      setElementArmState(ElementArmState.MANUAL);
      manualEnterRumbleCommand.start();
    }
  }

  @Override
  public void periodic() {
    BumbleTimer.start("ElementArm");
    if (getCurrentElementArmState() == ElementArmState.MANUAL) {
      manualControl();
    } else if (getCurrentElementArmState() == ElementArmState.SEMI_AUTOMATIC) {
      checkPotentiometer();
      toggleSemiAutomatic();
      updateGravityCompensationPower();
      semiAutomaticControl();
    } else {
      if (getCurrentElementArmState() != ElementArmState.CALIBRATION) {
        checkPotentiometer();
        toggleSemiAutomatic();
      }
      updateOnTarget();
      updateGravityCompensationPower();
      updatePidPresetAccordingToGamePiece();
      updatePidOutputRange();
    }
    BumbleTimer.time("ElementArm");
  }

  @Override
  public void sendDebuggingData() {
    SmartDashboard.putNumber("Debugging/ElementArm/Potentiometer Voltage", elementArmPotentiometer.getVoltage());
    SmartDashboard.putNumber("Debugging/ElementArm/ActualAngle", getActualAngle());
    SmartDashboard.putNumber("Debugging/ElementArm/RawAngle", elementArmPotentiometer.get());

    SmartDashboard.putBoolean("Debugging/ElementArm/isOnTarget", isOnTarget());
    SmartDashboard.putBoolean("Debugging/ElementArm/PID Enabled", pidController.isEnabled());
    SmartDashboard.putNumber("Debugging/ElementArm/Error", pidController.getError());

    double pidOutput = pidController.get();
    SmartDashboard.putNumber("Debugging/ElementArm/PID Output", pidOutput);
    SmartDashboard.putNumber("Debugging/ElementArm/Friction Overcome",
        Math.signum(pidOutput) * elementArmMotor.getFrictionOvercomePower());

    SmartDashboard.putNumber("Debugging/ElementArm/Gravity Compensation", -gravityCompensationPower);
    SmartDashboard.putNumber("Debugging/ElementArm/Motor GET Output", elementArmMotor.get());
    SmartDashboard.putNumber("Debugging/ElementArm/Motor getMotorOutputPercent",
        elementArmMotor.getMotorOutputPercent());

    SmartDashboard.putNumber("Debugging/ElementArm/Velocity", getCurrentVelocity());

    SmartDashboard.putString("Debugging/ElementArm/State", getCurrentElementArmState().toString());
  }

  public boolean getEjectingHatchPanel() {
    return isEjectingHatchPanel;
  }

  class ActualAnglePIDSource implements PIDSource {

    PIDSourceType pidSource = PIDSourceType.kDisplacement;

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
      this.pidSource = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return pidSource;
    }

    @Override
    public double pidGet() {
      return getActualAngle();
    }
  }
}

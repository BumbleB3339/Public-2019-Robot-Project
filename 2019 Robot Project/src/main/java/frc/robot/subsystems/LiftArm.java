/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bumblelib.util.BasePowerCalculator;
import frc.bumblelib.util.BumblePIDController;
import frc.bumblelib.util.BumbleTimer;
import frc.bumblelib.util.PIDPreset;
import frc.bumblelib.util.SmartdashboardDebugging;
import frc.bumblelib.util.StableBoolean;
import frc.bumblelib.util.VelocityKalmanFilter;
import frc.bumblelib.util.hardware.BasePowerCANSparkMax;
import frc.bumblelib.util.hardware.BumblePotentiometer;
import frc.robot.ControlledGamePieceDetector;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotGameStateManager;
import frc.robot.RobotMap;
import frc.robot.commands.controller.CommandTimedControllerRumble;
import frc.robot.commands.controller.CommandTimedControllerRumble.RumbleLength;
import frc.robot.subsystems.ElementArm.ElementArmState;

/**
 * Add your docs here.
 */
public class LiftArm extends Subsystem implements SmartdashboardDebugging {

  public BumblePotentiometer liftArmPotentiometer;
  public BasePowerCANSparkMax liftArmMotor;
  public BumblePIDController pidController;
  private double gravityCompensationPower = 0;
  private VelocityKalmanFilter velocityKalmanFilter = new VelocityKalmanFilter(20);

  private boolean softApproachUpEnabled = false;

  private double currentAngleSetpoint = -3339;
  private double desiredAngleSetpoint = -3339;

  private final double MIN_SAFE_DOWN_ELEMENT_ARM_ANGLE = 35;
  private final double MAX_SAFE_DOWN_ANGLE = 130;

  private final int ON_TARGET_TRUE_THRESHOLD = 10;
  private final int ON_TARGET_FALSE_THRESHOLD = 5;
  private StableBoolean onTarget = new StableBoolean(ON_TARGET_TRUE_THRESHOLD, ON_TARGET_FALSE_THRESHOLD);

  private final boolean IS_MOTOR_INVERTED = ROBOT_PROFILE.liftArmParams.isMotorInverted;
  private final IdleMode DEFAULT_IDLE_MODE = IdleMode.kBrake;
  private final int SMART_CURRENT_LIMIT = 80;

  private final double MANUAL_POWER_FORWARD = ROBOT_PROFILE.liftArmParams.manualPowerForward;
  private final double MANUAL_POWER_BACKWARD = ROBOT_PROFILE.liftArmParams.manualPowerBackward;

  private final LiftArmState DEFAULT_LIFT_ARM_STATE = LiftArmState.MANUAL;
  public final double RAMP_RATE = ROBOT_PROFILE.liftArmParams.rampRate;

  // On target params
  public double POSITION_TOLERANCE_ANGLE = 5;
  public double VELOCITY_TOLERANCE = 100000000;
  public double frictionOvercomePower = ROBOT_PROFILE.liftArmParams.frictionOvercomePower;

  public double GRAVITY_F_0_NONE = ROBOT_PROFILE.liftArmParams.gravityF_0_none;
  public double GRAVITY_F_90_NONE = ROBOT_PROFILE.liftArmParams.gravityF_90_none;
  public double GRAVITY_F_0_CARGO = ROBOT_PROFILE.liftArmParams.gravityF_0_cargo;
  public double GRAVITY_F_90_CARGO = ROBOT_PROFILE.liftArmParams.gravityF_90_cargo;
  public double GRAVITY_F_0_HATCH_PANEL = ROBOT_PROFILE.liftArmParams.gravityF_0_hatchPanel;
  public double GRAVITY_F_90_HATCH_PANEL = ROBOT_PROFILE.liftArmParams.gravityF_90_hatchPanel;

  public boolean isGravityFCalibrationMode = false;
  public double calibrationGravityF = 0.0;

  private static final double MAX_OUTPUT_UP_STRONG = ROBOT_PROFILE.liftArmParams.maxOutputUpStrong;
  private static final double MAX_OUTPUT_DOWN_STRONG = 0.60;
  private static final double MAX_OUTPUT_WEAK = 0.30;
  private static final double SOFT_APPROACH_SETPOINT_ACTIVE_RANGE = 70.0; // symmetric around zero
  private static final double SOFT_APPROACH_WEAK_RANGE = 70.0; // range of weak power around target
  private static final double SOFT_APPROACH_DISABLE_RANGE = 20.0;
  private static final double MAX_OUTPUT_ON_TARGET = 0.10;
  private static final double ZERO_OUTPUT_ON_TARGET_RANGE = 20.0; // symmetric around zero

  public PIDPreset pidPreset_none = ROBOT_PROFILE.liftArmParams.pidPreset_none;
  public PIDPreset pidPreset_cargo = ROBOT_PROFILE.liftArmParams.pidPreset_cargo;
  public PIDPreset pidPreset_hatch = ROBOT_PROFILE.liftArmParams.pidPreset_hatch;

  private LiftArmState currentLiftArmState = DEFAULT_LIFT_ARM_STATE;

  public enum LiftArmState {
    MANUAL, //
    CALIBRATION, //
    SEMI_AUTOMATIC, //
    FOLLOWING_CURRENT_STATE;

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

  public LiftArm() {
    liftArmMotor = new BasePowerCANSparkMax(RobotMap.LiftArmPorts.MOTOR, MotorType.kBrushless, frictionOvercomePower);
    liftArmMotor.restoreFactoryDefaults();
    liftArmMotor.setInverted(IS_MOTOR_INVERTED);
    liftArmMotor.setIdleMode(DEFAULT_IDLE_MODE);
    liftArmMotor.enableVoltageCompensation(12);
    liftArmMotor.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
    liftArmMotor.setOpenLoopRampRate(RAMP_RATE);

    liftArmPotentiometer = new BumblePotentiometer(RobotMap.LiftArmPorts.POTENTIOMETER, 0,
        ROBOT_PROFILE.liftArmParams.voltageInAngle0, 90, ROBOT_PROFILE.liftArmParams.voltageInAngle90,
        -ROBOT_PROFILE.robotStateParams.folded.liftArmAngle - 10,
        ROBOT_PROFILE.robotStateParams.folded.liftArmAngle + 10);

    pidController = new BumblePIDController(pidPreset_none.getKp(), pidPreset_none.getKi(), pidPreset_none.getKd(),
        new ActualAnglePIDSource(), liftArmMotor);
    pidController.disable();
    pidController.setAbsoluteTolerance(POSITION_TOLERANCE_ANGLE);
    pidController.setOutputRange(-MAX_OUTPUT_UP_STRONG, MAX_OUTPUT_UP_STRONG);
    setLiftArmState(DEFAULT_LIFT_ARM_STATE);
  }

  @Override
  public void initDefaultCommand() {
    // nothing
  }

  public void setPower(double power) {
    liftArmMotor.set(power);
  }

  public void setWithBasePower(double output) {
    setPower(BasePowerCalculator.calculateModifiedOutput(output, -gravityCompensationPower, frictionOvercomePower));
  }

  public void stop() {
    liftArmMotor.set(0);
  }

  public void manuaLiftArmForward() {
    liftArmMotor.set(MANUAL_POWER_FORWARD);
  }

  public void manualLiftArmBackward() {
    liftArmMotor.set(MANUAL_POWER_BACKWARD);
  }

  private void applyPIDPreset(PIDPreset pidPreset) {
    pidController.setPID(pidPreset.getKp(), pidPreset.getKi(), pidPreset.getKd());
  }

  public void applyPIDPreset(double kP, double kI, double kD) {
    pidController.setPID(kP, kI, kD);
  }

  public void setPIDSetpoint(double angleSetpoint) {
    if (getCurrentLiftArmState() == LiftArmState.MANUAL) {
      // don't allow changing PID setpoint on manual
      return;
    }

    if (desiredAngleSetpoint != angleSetpoint) {
      onTarget.forceValue(false);
    }

    this.desiredAngleSetpoint = angleSetpoint;

    // enable soft approach if the setpoint angle is in a given range
    if (Math.abs(angleSetpoint) <= SOFT_APPROACH_SETPOINT_ACTIVE_RANGE) {
      softApproachUpEnabled = true;
    }

  }

  private double lastCurrentAngleSetpoint = -3339.0;

  // This method calculates what the current setpoint on the way to the desired
  // setpoint should be, based on the state of other robot subsystems
  private void updateSetpoint() {
    if (getCurrentLiftArmState() == LiftArmState.CALIBRATION) {
      currentAngleSetpoint = desiredAngleSetpoint;
      if (currentAngleSetpoint != lastCurrentAngleSetpoint) {
        pidController.setSetpoint(currentAngleSetpoint);
      }
      lastCurrentAngleSetpoint = currentAngleSetpoint;
    }
    if (getCurrentLiftArmState() != LiftArmState.FOLLOWING_CURRENT_STATE) {
      return;
    }

    if (currentAngleSetpoint != desiredAngleSetpoint) {
      if (isSafeToMoveDown())
        currentAngleSetpoint = desiredAngleSetpoint;
      else {
        currentAngleSetpoint = Math.signum(desiredAngleSetpoint) * MAX_SAFE_DOWN_ANGLE;
      }
    }

    if (currentAngleSetpoint != lastCurrentAngleSetpoint) {
      pidController.setSetpoint(currentAngleSetpoint);
    }

    lastCurrentAngleSetpoint = currentAngleSetpoint;
  }

  private boolean isSafeToMoveDown() {
    if (desiredAngleSetpoint > MAX_SAFE_DOWN_ANGLE) { // LiftArm is aiming down forward
      if (Robot.m_elementArm.getActualAngle() > -MIN_SAFE_DOWN_ELEMENT_ARM_ANGLE) {
        return false;
      }
    } else if (desiredAngleSetpoint < -MAX_SAFE_DOWN_ANGLE) { // LiftArm is aiming down backward
      if (Robot.m_elementArm.getActualAngle() < MIN_SAFE_DOWN_ELEMENT_ARM_ANGLE) {
        return false;
      }
    }
    return true;
  }

  /**
   * @return the desiredAngleSetpoint
   */
  public double getDesiredAngleSetpoint() {
    return desiredAngleSetpoint;
  }

  private ElementArmState lastElementArmState;

  private void toggleSemiAutomatic() {
    if (getCurrentLiftArmState() == LiftArmState.MANUAL) {
      return;
    }

    ElementArmState currentElementArmState = Robot.m_elementArm.getCurrentElementArmState();

    if (getCurrentLiftArmState() != LiftArmState.SEMI_AUTOMATIC && currentElementArmState == ElementArmState.MANUAL) {
      // change to semi-automatic
      setLiftArmState(LiftArmState.SEMI_AUTOMATIC);
    } else if (getCurrentLiftArmState() == LiftArmState.SEMI_AUTOMATIC && lastElementArmState == ElementArmState.MANUAL
        && currentElementArmState != ElementArmState.MANUAL) {
      // change back to normal
      setLiftArmState(LiftArmState.SEMI_AUTOMATIC);
    }

    lastElementArmState = currentElementArmState;
  }

  private final double SEMT_AUTOMATIC_POWER_COEFF = 0.5;

  private void semiAutomaticControl() {
    if (getCurrentLiftArmState() != LiftArmState.SEMI_AUTOMATIC || !Robot.isOperationActive) {
      return;
    }

    if (Math.abs(OI.operatorController.getY(Hand.kLeft)) > 0.2) {
      setWithBasePower(OI.operatorController.getY(Hand.kLeft) * SEMT_AUTOMATIC_POWER_COEFF);
    } else {
      setWithBasePower(0.0);
    }
  }

  public void setLiftArmState(LiftArmState liftArmState) {
    this.currentLiftArmState = liftArmState;
    if (liftArmState == LiftArmState.FOLLOWING_CURRENT_STATE) {
      pidController.enable();
    } else {
      pidController.disable();
    }
    if (liftArmState == LiftArmState.SEMI_AUTOMATIC) {
      RobotGameStateManager.setRobotSystemStateToSemiAuto();
    }
  }

  public double getActualAngle() {
    if (this.liftArmPotentiometer.get() < 0) {
      return this.liftArmPotentiometer.get() - ROBOT_PROFILE.liftArmParams.angleFreedom;
    }
    return this.liftArmPotentiometer.get();
  }

  private final int ITERATIONS_TO_DETERMINE_DEFECTIVE = 15;
  private int defectiveCounter = 0;
  private boolean lastIsPotentiometerDefective = false;
  private boolean isPotentiometerDefective = false;

  private void checkPotentiometer() {
    defectiveCounter = liftArmPotentiometer.isDefective() ? defectiveCounter + 1 : 0;
    boolean isDefective = defectiveCounter >= ITERATIONS_TO_DETERMINE_DEFECTIVE;
    isPotentiometerDefective = isDefective;
    if (isDefective && !lastIsPotentiometerDefective) {
      setLiftArmState(LiftArmState.MANUAL);
      DriverStation.reportError("LiftArm potentiometer out of range, LiftArm is set to MANUAL", false);
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
    onTarget.update(pidController.onTarget()); // && Math.abs(getCurrentVelocity()) < VELOCITY_TOLERANCE
  }

  public boolean isOnTarget() {
    return getCurrentLiftArmState() == LiftArmState.MANUAL || onTarget.get();
  }

  /**
   * This methods periodically calculates the base power to the arm PID loop and
   * applies it. It is derived from:
   * https://drive.google.com/file/d/1FUZyMC-_ZrIn2FgOAd0_Q0TUYOHEyPpA/view?usp=sharing
   */
  private void updateGravityCompensationPower() {
    double gravity_F;
    if (isGravityFCalibrationMode) {
      gravity_F = calibrationGravityF;
    } else {
      gravity_F = getGravityF();
    }

    gravityCompensationPower = gravity_F * Math.sin(Math.toRadians(getActualAngle()));
    liftArmMotor.setGravityCompensationPower(-gravityCompensationPower);

    // double sinElementArmAbsoluteAngle =
    // Robot.m_elementArm.getCurrentElementArmState() == ElementArmState.MANUAL
    // ? 0.5 : Math.sin(Math.toRadians(Robot.m_elementArm.getAbsoluteAngle()));
    // gravityCompensationPower = (F_2_POWER - FRICTION_OVERCOME_POWER) *
    // Math.sin(Math.toRadians(getActualAngle()))
    // + (F_1_POWER - F_2_POWER) * sinElementArmAbsoluteAngle;
  }

  private double getGravityF() {
    double gravityF_0, gravityF_90;
    switch (ControlledGamePieceDetector.getControlledGamePiece()) {
    case CARGO:
      gravityF_0 = GRAVITY_F_0_CARGO;
      gravityF_90 = GRAVITY_F_90_CARGO;
      break;
    case HATCH_PANEL:
      gravityF_0 = GRAVITY_F_0_HATCH_PANEL;
      gravityF_90 = GRAVITY_F_90_HATCH_PANEL;
      break;
    case NONE:
    default:
      gravityF_0 = GRAVITY_F_0_NONE;
      gravityF_90 = GRAVITY_F_90_NONE;
      break;
    }

    return gravityF_0 - ((gravityF_0 - gravityF_90) * (Math.abs(Robot.m_elementArm.getActualAngle()) / 90.0));
  }

  public void updatePidMaxOutput() {
    if (isOnTarget()) {
      // limit to small values when on target, unless if inside small range around 0
      double absAngle = Math.abs(getActualAngle());
      if (absAngle <= ZERO_OUTPUT_ON_TARGET_RANGE || absAngle >= 150) {
        pidController.setOutputRange(0.0, 0.0);
      } else {
        pidController.setOutputRange(-MAX_OUTPUT_ON_TARGET, MAX_OUTPUT_ON_TARGET);
      }
    } else {
      double actualAngle = getActualAngle();

      if (softApproachUpEnabled) {
        double angularAbsError = Math.abs(actualAngle - currentAngleSetpoint);
        double maxOutputUp, maxOutputDown;

        // limit up direction to prevent hard stop on the top
        if (angularAbsError <= SOFT_APPROACH_WEAK_RANGE) {
          maxOutputUp = MAX_OUTPUT_WEAK;
          maxOutputDown = MAX_OUTPUT_WEAK;
        } else {
          maxOutputUp = MAX_OUTPUT_UP_STRONG;
          maxOutputDown = MAX_OUTPUT_DOWN_STRONG;
        }

        if (actualAngle > 0) {
          pidController.setOutputRange(-maxOutputUp, maxOutputDown);
        } else {
          pidController.setOutputRange(-maxOutputDown, maxOutputUp);
        }

        if (angularAbsError <= SOFT_APPROACH_DISABLE_RANGE) {
          softApproachUpEnabled = false;
        }
      } else {
        // limit up and down movement differently
        if (actualAngle > 0) {
          pidController.setOutputRange(-MAX_OUTPUT_UP_STRONG, MAX_OUTPUT_DOWN_STRONG);
        } else {
          pidController.setOutputRange(-MAX_OUTPUT_DOWN_STRONG, MAX_OUTPUT_UP_STRONG);
        }
      }
    }
  }

  private static PIDPreset lastUpdatedPidPreset = null;

  private void updatePidPresetAccordingToGamePiece() {
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

  public LiftArmState getCurrentLiftArmState() {
    return currentLiftArmState;
  }

  private void manualControl() {
    if (getCurrentLiftArmState() != LiftArmState.MANUAL || !Robot.isOperationActive) {
      return;
    }

    if (Math.abs(OI.operatorController.getY(Hand.kLeft)) > 0.2) {
      liftArmMotor.set(OI.operatorController.getY(Hand.kLeft) * 0.8);
    } else {
      liftArmMotor.set(0.0);
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

    if (getCurrentLiftArmState() == LiftArmState.MANUAL) {
      setLiftArmState(LiftArmState.SEMI_AUTOMATIC);
      manualFinishRumbleCommand.start();
    } else {
      setLiftArmState(LiftArmState.MANUAL);
      manualEnterRumbleCommand.start();
    }
  }

  @Override
  public void periodic() {
    BumbleTimer.start("LiftArm");
    if (getCurrentLiftArmState() == LiftArmState.MANUAL) {
      manualControl();
    } else if (getCurrentLiftArmState() == LiftArmState.SEMI_AUTOMATIC) {
      checkPotentiometer();
      toggleSemiAutomatic();
      updateGravityCompensationPower();
      semiAutomaticControl();
    } else {
      if (getCurrentLiftArmState() != LiftArmState.CALIBRATION) {
        checkPotentiometer();
        toggleSemiAutomatic();
      }
      updateSetpoint();
      updateOnTarget();
      updateGravityCompensationPower();
      updatePidPresetAccordingToGamePiece();
      updatePidMaxOutput();
    }
    BumbleTimer.time("LiftArm");
  }

  @Override
  public void sendDebuggingData() {
    SmartDashboard.putNumber("Debugging/LiftArm/Potentiometer Voltage", liftArmPotentiometer.getVoltage());
    SmartDashboard.putNumber("Debugging/LiftArm/Actual Angle", getActualAngle());
    SmartDashboard.putNumber("Debugging/LiftArm/Raw Angle", liftArmPotentiometer.get());

    SmartDashboard.putBoolean("Debugging/LiftArm/isOnTarget", isOnTarget());
    SmartDashboard.putBoolean("Debugging/LiftArm/PID Enabled", pidController.isEnabled());
    SmartDashboard.putNumber("Debugging/LiftArm/Error", pidController.getError());

    double pidOutput = pidController.get();
    SmartDashboard.putNumber("Debugging/LiftArm/PID Output", pidOutput);
    SmartDashboard.putNumber("Debugging/LiftArm/Friction Overcome",
        Math.signum(pidOutput) * liftArmMotor.getFrictionOvercomePower());

    SmartDashboard.putNumber("Debugging/LiftArm/Gravity Compensation", -gravityCompensationPower);
    SmartDashboard.putNumber("Debugging/LiftArm/Motor Get Output", liftArmMotor.get());
    SmartDashboard.putNumber("Debugging/LiftArm/Motor Applied Output", liftArmMotor.getAppliedOutput());

    SmartDashboard.putNumber("Debugging/LiftArm/Velocity", getCurrentVelocity());

    SmartDashboard.putBoolean("Debugging/LiftArm/softApproachUpEnabled", softApproachUpEnabled);

    SmartDashboard.putString("Debugging/LiftArm/State", getCurrentLiftArmState().toString());
    SmartDashboard.putNumber("Debugging/LiftArm/Motor Current", liftArmMotor.getOutputCurrent());
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

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
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bumblelib.util.BasePowerCalculator;
import frc.bumblelib.util.BumbleTimer;
import frc.bumblelib.util.PIDPreset;
import frc.bumblelib.util.SmartdashboardDebugging;
import frc.bumblelib.util.StableBoolean;
import frc.bumblelib.util.VelocityKalmanFilter;
import frc.bumblelib.util.hardware.BasePowerWPI_TalonSRX;
import frc.bumblelib.util.hardware.BumblePotentiometer;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameState.RobotSystemState;
import frc.robot.RobotGameStateManager;
import frc.robot.RobotMap;
import frc.robot.commands.controller.CommandTimedControllerRumble;
import frc.robot.commands.controller.CommandTimedControllerRumble.RumbleLength;

/**
 * Add your docs here.
 */
public class IntakeArm extends Subsystem implements SmartdashboardDebugging {

  private final IntakeArmState DEFAULT_INTAKE_ARM_STATE = IntakeArmState.MANUAL;
  private final boolean IS_MOTOR_INVERTED = ROBOT_PROFILE.intakeArmParams.isMotorInverted;
  private final NeutralMode DEFAULT_NEUTRAL_MODE = NeutralMode.Brake;
  private double gravityCompensationPower = 0;
  private VelocityKalmanFilter velocityKalmanFilter = new VelocityKalmanFilter(10);

  private final double MAXIMUM_LIFT_ARM_ANGLE_TO_MOVE = 90;

  private final int ON_TARGET_TRUE_THRESHOLD = 5;
  private final int ON_TARGET_FALSE_THRESHOLD = 5;
  private StableBoolean onTarget = new StableBoolean(ON_TARGET_TRUE_THRESHOLD, ON_TARGET_FALSE_THRESHOLD);

  public BumblePotentiometer intakeArmPotentiometer;
  public BasePowerWPI_TalonSRX intakeArmMotor; // Should be public to access its sensor from another classes.
  public PIDController pidController;

  public boolean isGravityFCalibrationMode = false;
  public double calibrationGravityF = 0.0;

  private final double MANUAL_POWER_FORWARD = ROBOT_PROFILE.elementArmParams.manualPowerForward;
  private final double MANUAL_POWER_BACKWARD = ROBOT_PROFILE.elementArmParams.manualPowerBackward;

  // On target params
  public double POSITION_TOLERANCE_ANGLE = 3;
  public double VELOCITY_TOLERANCE = 100000000;

  private final PIDPreset PID_PRESET = ROBOT_PROFILE.elementArmParams.pidPreset_none;
  public double frictionOvercomePower = ROBOT_PROFILE.intakeArmParams.frictionOvercomePower;
  public double GRAVITY_F = ROBOT_PROFILE.intakeArmParams.fPower;
  private final double ANGLE_TO_BALANCED = ROBOT_PROFILE.intakeArmParams.angleToBalanced;

  public double RAMP_RATE = 0.1;

  private double currentAngleSetpoint = 0;
  private IntakeArmState currentIntakeArmState = DEFAULT_INTAKE_ARM_STATE;
  private IntakeArmState desiredIntakeArmState = DEFAULT_INTAKE_ARM_STATE;

  private static final double WAITING_TO_ENTER_ANGLE = 85;
  private static final double WAITING_TO_ENTER_ANGLE_TOLERANCE = 20;
  private static final double APPROXIMATELY_ON_TARGET_TOLERANCE = 15;
  private static final double FOLDED_ZERO_POWER_TOLERANCE = 5;

  private static final double CONSTANT_FOLD_POWER = 1; // must be positive
  private static final double CONSTANT_FOLD_ANGLE = 40;

  private static final double CONSTANT_HATCH_COLLECT_POWER = 0.1;
  private static final double CONSTANT_HATCH_COLLECT_ANGLE_TOLERANCE = 3;

  public enum IntakeArmState {
    MANUAL(-3339), //
    CALIBRATION(-3339), //
    CLIMB(-3339), //
    FOLDED(ROBOT_PROFILE.intakeArmParams.foldedAngle), //
    STAY_IN_PLACE(ROBOT_PROFILE.intakeArmParams.foldedAngle), // set to folded just to prevent bad behaviour
    WAITING_TO_ENTER(WAITING_TO_ENTER_ANGLE), //
    COLLECT_CARGO(ROBOT_PROFILE.intakeArmParams.collectCargoAngle), //
    COLLECT_HATCH(ROBOT_PROFILE.intakeArmParams.collectHatchAngle), //
    DELIVER_HATCH(ROBOT_PROFILE.intakeArmParams.deliverHatchPanelAngle), //
    DELIVER_HATCH_PRESSURE(ROBOT_PROFILE.intakeArmParams.deliverHatchPanelAngle - 5), //
    PREPARE_TO_CLIMB(ROBOT_PROFILE.intakeArmParams.prepareToClimbAngle), //
    ABORT_CARGO_FLOOR_COLLECT(110), //
    SAFETY_HATCH_EJECT(80), //
    LEVEL2_PREPARE_TO_CLIMB(20);

    private final double angle;

    IntakeArmState(double angle) {
      this.angle = angle;
    }

    public double getAngle() {
      return angle;
    }

    public String toDashboardString() {
      switch (this) {
      case MANUAL:
        return "Manual";
      case CALIBRATION:
        return "Calibration";
      default:
        return "Automatic";
      }
    }
  }

  public IntakeArm() {
    intakeArmMotor = new BasePowerWPI_TalonSRX(RobotMap.IntakeArmPorts.MOTOR, frictionOvercomePower);
    intakeArmMotor.setInverted(IS_MOTOR_INVERTED);
    intakeArmMotor.setNeutralMode(DEFAULT_NEUTRAL_MODE);
    intakeArmMotor.configVoltageCompSaturation(11);
    intakeArmMotor.enableVoltageCompensation(true);
    intakeArmMotor.configOpenloopRamp(RAMP_RATE);

    intakeArmPotentiometer = new BumblePotentiometer(RobotMap.IntakeArmPorts.POTENTIMETER, 0,
        ROBOT_PROFILE.intakeArmParams.voltageInAngle0, 90, ROBOT_PROFILE.intakeArmParams.voltageInAngle90,
        IntakeArmState.FOLDED.getAngle() - 10, 180.0 + 20.0);

    pidController = new PIDController(PID_PRESET.getKp(), PID_PRESET.getKi(), PID_PRESET.getKd(),
        intakeArmPotentiometer, intakeArmMotor);
    setPidEnable(false);
    pidController.setAbsoluteTolerance(POSITION_TOLERANCE_ANGLE);
    setPosition(DEFAULT_INTAKE_ARM_STATE);
  }

  public double getCurrentSetpoint() {
    return pidController.getSetpoint();
  }

  @Override
  public void initDefaultCommand() {
    // nothing
  }

  public void stop() {
    intakeArmMotor.set(0);
  }

  public void setPower(double power) {
    intakeArmMotor.set(power);
  }

  public void manualElementArmForward() {
    intakeArmMotor.set(MANUAL_POWER_FORWARD);
  }

  public void manualElementArmBackward() {
    intakeArmMotor.set(MANUAL_POWER_BACKWARD);
  }

  private void applyPIDPreset(PIDPreset pidPreset) {
    pidController.setPID(pidPreset.getKp(), pidPreset.getKi(), pidPreset.getKd());
  }

  public void applyPIDPreset(double kP, double kI, double kD) {
    pidController.setPID(kP, kI, kD);
  }

  private boolean isPidEnabled = false;

  private void setPidEnable(boolean enable) {
    if (enable && !isPidEnabled) {
      pidController.enable();
      isPidEnabled = true;
    } else if (!enable && isPidEnabled) {
      pidController.disable();
      isPidEnabled = false;
    }
  }

  public void setPIDSetpoint(double angleSetpoint) {
    if (getCurrentIntakeArmState() == IntakeArmState.MANUAL) {
      // don't allow changing PID setpoint on manual
      return;
    }

    if (angleSetpoint != this.currentAngleSetpoint) {
      // reset OnTraget if the target was changed
      onTarget.forceValue(false);
      pidController.setSetpoint(angleSetpoint);

      // This sets back the neutral mode to Brake because it might be
      // in Coast during HATCH_COLLECT
      // intakeArmMotor.setNeutralMode(NeutralMode.Brake);
    }

    this.currentAngleSetpoint = angleSetpoint;

    setPidEnable(true);
  }

  private double getTargetAngle() {
    return desiredIntakeArmState.getAngle();
  }

  public boolean isInsideRobot(boolean useTolerance) {
    double angleThreshold = WAITING_TO_ENTER_ANGLE;
    if (useTolerance) {
      angleThreshold -= WAITING_TO_ENTER_ANGLE_TOLERANCE;
    }
    return getActualAngle() <= angleThreshold;
  }

  public boolean isInsideRobot() {
    return isInsideRobot(false);
  }

  public boolean isOutsideRobot(boolean useTolerance) {
    double angleThreshold = WAITING_TO_ENTER_ANGLE;
    if (useTolerance) {
      angleThreshold -= WAITING_TO_ENTER_ANGLE_TOLERANCE;
    }
    return getActualAngle() >= angleThreshold;
  }

  public boolean isOutsideRobot() {
    return isOutsideRobot(false);
  }

  private boolean isAllowedToMoveIn() {
    double minLiftHeightToMove;
    if (RobotGameStateManager.currentGameState.robotSystemState == RobotSystemState.FRONT_INTAKE_ARM_GAP_LOW) {
      minLiftHeightToMove = RobotSystemState.FRONT_INTAKE_ARM_GAP_LOW.robotConfiguration.liftHeight - 1;
    } else if (RobotGameStateManager.currentGameState.robotSystemState == RobotSystemState.FLOOR_HATCH_END) {
      minLiftHeightToMove = RobotSystemState.FLOOR_HATCH_END.robotConfiguration.liftHeight - 1;
    } else {
      minLiftHeightToMove = RobotSystemState.FRONT_INTAKE_ARM_GAP_HIGH.robotConfiguration.liftHeight - 3;
    }
    return Robot.m_lift.getCurrentHeight() >= minLiftHeightToMove
        || (Robot.m_liftArm.getActualAngle() <= MAXIMUM_LIFT_ARM_ANGLE_TO_MOVE
            && Robot.m_liftArm.getDesiredAngleSetpoint() <= MAXIMUM_LIFT_ARM_ANGLE_TO_MOVE);
  }

  private boolean needsSpaceToMoveIn() {
    // intake is outside and wants to move in
    return getTargetAngle() <= WAITING_TO_ENTER_ANGLE && isOutsideRobot(true);
  }

  public void updateSetpointToAvoidCollision() {
    if (this.desiredIntakeArmState == IntakeArmState.MANUAL || this.desiredIntakeArmState == IntakeArmState.CALIBRATION
        || this.desiredIntakeArmState == IntakeArmState.CLIMB) {
      return;
    }

    if (this.desiredIntakeArmState == IntakeArmState.DELIVER_HATCH
        || this.desiredIntakeArmState == IntakeArmState.SAFETY_HATCH_EJECT
        || this.desiredIntakeArmState == IntakeArmState.DELIVER_HATCH_PRESSURE
        || this.desiredIntakeArmState == IntakeArmState.PREPARE_TO_CLIMB
        || RobotGameStateManager.currentGameState.robotAction == RobotAction.CLIMB) {
      this.currentIntakeArmState = this.desiredIntakeArmState;
    } else if (!isAllowedToMoveIn()) { // intake not allowed to move freely
      if (isApproximatelyFolded(false)) {
        this.currentIntakeArmState = IntakeArmState.FOLDED;
      } else if (isInsideRobot(true)) {
        this.currentIntakeArmState = IntakeArmState.STAY_IN_PLACE;
      } else if (needsSpaceToMoveIn()) {
        this.currentIntakeArmState = IntakeArmState.WAITING_TO_ENTER;
      } else {
        this.currentIntakeArmState = this.desiredIntakeArmState;
      }
    } else {
      // allow to move freely
      this.currentIntakeArmState = this.desiredIntakeArmState;
    }

    if (this.currentIntakeArmState == IntakeArmState.STAY_IN_PLACE) {
      setPIDSetpoint(getActualAngle()); // don't change position until allowed
    } else if (this.currentIntakeArmState == IntakeArmState.FOLDED) {
      // when folding set constant power to increase speed
      if (getActualAngle() >= IntakeArmState.FOLDED.getAngle() + CONSTANT_FOLD_ANGLE) {
        setPidEnable(false);
        intakeArmMotor.set(-CONSTANT_FOLD_POWER);
      } else if (getActualAngle() <= IntakeArmState.FOLDED.getAngle() + FOLDED_ZERO_POWER_TOLERANCE) {
        setPidEnable(false);
        intakeArmMotor.set(0.0);
      } else {
        setPIDSetpoint(IntakeArmState.FOLDED.getAngle());
      }
    } else if (this.currentIntakeArmState == IntakeArmState.COLLECT_CARGO) {
      if (isInsideRobot()) {
        setPidEnable(false);
        intakeArmMotor.set(CONSTANT_FOLD_POWER);
      } else {
        setPIDSetpoint(IntakeArmState.COLLECT_CARGO.getAngle());
      }
    } else if (this.currentIntakeArmState == IntakeArmState.COLLECT_HATCH) {
      // double anglularDistance = Math.abs(getActualAngle() -
      // IntakeArmState.COLLECT_HATCH.getAngle());
      // if (anglularDistance <= CONSTANT_HATCH_COLLECT_ANGLE_TOLERANCE) {
      // setPidEnable(false);
      // intakeArmMotor.setNeutralMode(NeutralMode.Coast);
      // intakeArmMotor.set(0.0);
      // } else {
      setPIDSetpoint(IntakeArmState.COLLECT_HATCH.getAngle());
      // }
    } else {
      setPIDSetpoint(this.currentIntakeArmState.getAngle());
    }
  }

  public void setPosition(IntakeArmState intakeArmState) {
    if (desiredIntakeArmState != intakeArmState) {
      onTarget.forceValue(false);
    }
    // IMPORTANT! PID setpoint and currentState is changed by
    // the function updateSetpointToAvoidCollision
    this.desiredIntakeArmState = intakeArmState;

    // disable PID for these states
    if (intakeArmState == IntakeArmState.MANUAL || intakeArmState == IntakeArmState.CALIBRATION
        || intakeArmState == IntakeArmState.CLIMB) {
      this.currentIntakeArmState = intakeArmState;
      setPidEnable(false);
    }
  }

  public void resetToFolded() {
    this.desiredIntakeArmState = IntakeArmState.FOLDED;
    this.currentIntakeArmState = IntakeArmState.FOLDED;
  }

  public void setManualMode(boolean enable) {
    if (enable) {
      setPosition(IntakeArmState.MANUAL);
    } else if (!intakeArmPotentiometer.isDefective()) {
      currentIntakeArmState = DEFAULT_INTAKE_ARM_STATE;
      setPosition(currentIntakeArmState);
    }
  }

  public double getActualAngle() {
    return this.intakeArmPotentiometer.get();
  }

  private final int ITERATIONS_TO_DETERMINE_DEFECTIVE = 15;
  private int defectiveCounter = 0;
  private boolean lastIsPotentiometerDefective = false;
  private boolean isPotentiometerDefective = false;

  private void checkPotentiometer() {
    defectiveCounter = intakeArmPotentiometer.isDefective() ? defectiveCounter + 1 : 0;
    boolean isDefective = defectiveCounter >= ITERATIONS_TO_DETERMINE_DEFECTIVE;
    isPotentiometerDefective = isDefective;
    if (isDefective && !lastIsPotentiometerDefective) {
      setManualMode(true);
      DriverStation.reportError("IntakeArm potentiometer out of range, IntakeArm is set to MANUAL", false);
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
    onTarget.update(pidController.onTarget() && (this.currentIntakeArmState == this.desiredIntakeArmState));
  }

  public boolean isOnTarget() {
    return getCurrentIntakeArmState() == IntakeArmState.MANUAL || onTarget.get();
  }

  public boolean isApproximatelyFolded(boolean checkFoldedState) {
    return (!checkFoldedState || this.currentIntakeArmState == IntakeArmState.FOLDED)
        && getActualAngle() <= IntakeArmState.FOLDED.getAngle() + APPROXIMATELY_ON_TARGET_TOLERANCE;
  }

  public boolean isApproximatelyFolded() {
    return isApproximatelyFolded(true);
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
      gravityF = GRAVITY_F;
    }

    gravityCompensationPower = gravityF * Math.sin(Math.toRadians(getActualAngle() - ANGLE_TO_BALANCED));
    intakeArmMotor.setGravityCompensationPower(-gravityCompensationPower);
  }

  public IntakeArmState getCurrentIntakeArmState() {
    return currentIntakeArmState;
  }

  private void manualControl() {
    if (getCurrentIntakeArmState() != IntakeArmState.MANUAL || !Robot.isOperationActive) {
      return;
    }

    if (Math.abs(OI.driverController.getY(Hand.kRight)) > 0.2) {
      intakeArmMotor.set(-OI.driverController.getY(Hand.kRight));
    } else {
      intakeArmMotor.set(0.0);
    }
  }

  private CommandTimedControllerRumble manualEnterRumbleCommand;
  private CommandTimedControllerRumble manualFinishRumbleCommand;

  public void toggleManual() {
    if (manualEnterRumbleCommand == null) {
      manualEnterRumbleCommand = new CommandTimedControllerRumble(OI.driverController, RumbleType.kRightRumble,
          RumbleLength.LONG, 1.0);
    }
    if (manualFinishRumbleCommand == null) {
      manualFinishRumbleCommand = new CommandTimedControllerRumble(OI.driverController, RumbleType.kRightRumble,
          RumbleLength.SHORT, 1.0);
    }

    if (getCurrentIntakeArmState() == IntakeArmState.MANUAL) {
      resetToFolded();
      manualFinishRumbleCommand.start();
    } else {
      setPosition(IntakeArmState.MANUAL);
      manualEnterRumbleCommand.start();
    }
  }

  @Override
  public void periodic() {
    BumbleTimer.start("IntakeArm");
    if (getCurrentIntakeArmState() == IntakeArmState.MANUAL) {
      manualControl();
    } else {
      if (getCurrentIntakeArmState() != IntakeArmState.CALIBRATION) {
        checkPotentiometer();
      }
      updateSetpointToAvoidCollision();
      updateOnTarget();
      updateGravityCompensationPower();
    }
    BumbleTimer.time("IntakeArm");
  }

  @Override
  public void sendDebuggingData() {
    SmartDashboard.putNumber("Debugging/IntakeArm/Potentiometer Voltage", intakeArmPotentiometer.getVoltage());
    SmartDashboard.putNumber("Debugging/IntakeArm/Angle", getActualAngle());
    SmartDashboard.putNumber("Debugging/IntakeArm/Error", pidController.getError());

    SmartDashboard.putBoolean("Debugging/IntakeArm/isOnTarget", isOnTarget());
    SmartDashboard.putBoolean("Debugging/IntakeArm/PID Enabled", pidController.isEnabled());

    double pidOutput = pidController.get();
    SmartDashboard.putNumber("Debugging/IntakeArm/PID Output", pidOutput);
    SmartDashboard.putNumber("Debugging/IntakeArm/Friction Overcome", Math.signum(pidOutput) * frictionOvercomePower);

    SmartDashboard.putNumber("Debugging/IntakeArm/Gravity Compensation", -gravityCompensationPower);
    SmartDashboard.putNumber("Debugging/IntakeArm/Motor GET Output", intakeArmMotor.get());
    SmartDashboard.putNumber("Debugging/IntakeArm/Motor getMotorOutputPercent", intakeArmMotor.getMotorOutputPercent());

    SmartDashboard.putNumber("Debugging/IntakeArm/Velocity", getCurrentVelocity());

    SmartDashboard.putBoolean("Debugging/IntakeArm/isAllowedToMoveIn", isAllowedToMoveIn());

    SmartDashboard.putString("Debugging/IntakeArm/CurrentState", this.currentIntakeArmState.toString());
    SmartDashboard.putString("Debugging/IntakeArm/DesiredState", this.desiredIntakeArmState.toString());
  }
}

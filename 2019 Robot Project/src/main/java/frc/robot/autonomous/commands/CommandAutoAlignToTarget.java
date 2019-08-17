/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous.commands;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.bumblelib.util.StableBoolean;
import frc.robot.Robot;
import frc.robot.RobotGameState.FieldObjective;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameStateManager;
import frc.robot.commands.CommandAlignToTarget;
import frc.robot.subsystems.Drivetrain.DrivetrainControlState;
import frc.robot.vision.MergeTargets;

public class CommandAutoAlignToTarget extends CommandAlignToTarget {

  public static final double MAX_Y_TO_COLLECT_HATCH = 0.48;
  private final double MAX_Y_TO_ROCKET_HATCH = 0.54;
  private final double MAX_Y_TO_CARGOSHIP_HATCH = 0.54;
  public static final double MIN_CURRENT_FOR_HATCH_COLLECT = 15.0;

  private boolean forceStoppingOnEncoders = false;
  private Side forcedSideMergingSide;
  private boolean isForcedSideMerging = false;

  private StableBoolean feederDistanceBoolean = new StableBoolean(4, 1);
  private StableBoolean cargoshipDistanceBoolean = new StableBoolean(4, 1);
  private StableBoolean rocketDistanceBoolean = new StableBoolean(4, 1);

  public CommandAutoAlignToTarget() {
    super(false);
  }

  public CommandAutoAlignToTarget(Side forcedSideMergingSide) {
    super(false);
    this.forcedSideMergingSide = forcedSideMergingSide;
    isForcedSideMerging = true;
  }

  public CommandAutoAlignToTarget(boolean forceStoppingOnEncoders) {
    super(false);
    this.forceStoppingOnEncoders = forceStoppingOnEncoders;
  }

  @Override
  protected void initialize() {
    super.initialize();
    Robot.m_drivetrain.initDrivetrainControlState(DrivetrainControlState.OPEN_LOOP);
    MergeTargets.isForcedSideMerging = isForcedSideMerging;
    MergeTargets.forcedSideMergingSide = forcedSideMergingSide;
    feederDistanceBoolean.forceValue(false);
    cargoshipDistanceBoolean.forceValue(false);
    rocketDistanceBoolean.forceValue(false);
  }

  @Override
  protected boolean isFinished() {
    // return (super.alignToTargetManeuver.getProximityModeTimeStamp() != -3339.0
    // && Timer.getFPGATimestamp() -
    // super.alignToTargetManeuver.getProximityModeTimeStamp() >
    // TIME_AFTER_PROXIMITY)
    // || (Robot.m_drivetrain.isDrivetrainStopped() && super.isInProximityMode());

    feederDistanceBoolean.update(super.getYCoordinate() != -3339 && super.getYCoordinate() < MAX_Y_TO_COLLECT_HATCH);
    rocketDistanceBoolean.update(super.getYCoordinate() != -3339 && super.getYCoordinate() < MAX_Y_TO_ROCKET_HATCH);
    cargoshipDistanceBoolean
        .update(super.getYCoordinate() != -3339 && super.getYCoordinate() < MAX_Y_TO_CARGOSHIP_HATCH);

    if (forceStoppingOnEncoders) {
      return (Robot.m_drivetrain.isDrivetrainStopped() && super.isInProximityMode());
    }
    if (RobotGameStateManager.currentGameState.robotAction == RobotAction.FEEDER_COLLECT) {
      return (feederDistanceBoolean.get()) || (Robot.m_drivetrain.isDrivetrainStopped() && super.isInProximityMode())
          || (super.isInProximityMode() && (Robot.m_drivetrain.getAverageCurrent() > MIN_CURRENT_FOR_HATCH_COLLECT));
    } else if (RobotGameStateManager.currentGameState.fieldObjective == FieldObjective.ROCKET) {
      return (rocketDistanceBoolean.get()) || (Robot.m_drivetrain.isDrivetrainStopped() && super.isInProximityMode());
    } else if (RobotGameStateManager.currentGameState.fieldObjective == FieldObjective.CARGOSHIP) {
      return (cargoshipDistanceBoolean.get())
          || (Robot.m_drivetrain.isDrivetrainStopped() && super.isInProximityMode());
    } else {
      return (Robot.m_drivetrain.isDrivetrainStopped() && super.isInProximityMode());
    }

    // return v || (super.getYCoordinate() != -3339 && super.getYCoordinate() <
    // MAX_Y_TO_COLLECT_HATCH);
  }

  @Override
  protected void end() {
    super.end();
    Robot.m_drivetrain.initDrivetrainControlState(DrivetrainControlState.PATH_FOLLOWING);
    MergeTargets.isForcedSideMerging = false;
  }
}

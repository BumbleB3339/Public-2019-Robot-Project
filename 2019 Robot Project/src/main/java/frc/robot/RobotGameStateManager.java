/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ControlledGamePieceDetector.ControlledGamePiece;
import frc.robot.RobotGameState.Direction;
import frc.robot.RobotGameState.RobotAction;
import frc.robot.RobotGameState.RobotSystemState;
import frc.robot.subsystems.ElementArm.ElementArmState;
import frc.robot.subsystems.Lift.LiftState;
import frc.robot.subsystems.LiftArm.LiftArmState;

/**
 * Add your docs here.
 */
public class RobotGameStateManager {

    private enum ArmOrientation {
        FROM_ABOVE, FROM_BELOW;
    }

    public enum ClimbMode {
        LEVEL2, LEVEL3;
    }

    public static RobotGameState currentGameState = new RobotGameState();
    public static RobotGameState nextGameState = new RobotGameState();
    public static ClimbMode climbMode = ClimbMode.LEVEL3;

    // the last robot game state that was sent to the systems as pid values
    private static RobotGameState.RobotSystemState lastUpdatedRobotSystemState;
    private static RobotGameState.Direction lastUpdatedDirection;

    public static void periodic() {
        if (!Robot.isOperationActive) {
            return;
        }

        if ((currentGameState.robotSystemState != lastUpdatedRobotSystemState
                || currentGameState.direction != lastUpdatedDirection) && !isAnySystemManual()) {

            // change setpoints only if not during semi automatic control
            if (currentGameState.robotSystemState != RobotSystemState.SEMI_AUTOMATIC) {
                Robot.m_lift.setPIDSetpoint(currentGameState.robotSystemState.robotConfiguration.liftHeight);
                Robot.m_liftArm.setPIDSetpoint((currentGameState.direction == Direction.FORWARD ? 1 : -1)
                        * currentGameState.robotSystemState.robotConfiguration.liftArmAngle);
                Robot.m_elementArm.setPIDSetpoint((currentGameState.direction == Direction.FORWARD ? 1 : -1)
                        * currentGameState.robotSystemState.robotConfiguration.elementArmAngle);

                setSystemsToFollowCurrentState();
            }

            lastUpdatedRobotSystemState = currentGameState.robotSystemState;
            lastUpdatedDirection = currentGameState.direction;
        }
    }

    public static boolean isAnySystemManual() {
        return Robot.m_liftArm.getCurrentLiftArmState() == LiftArmState.MANUAL
                || Robot.m_elementArm.getCurrentElementArmState() == ElementArmState.MANUAL
                || Robot.m_lift.getCurrentLiftState() == LiftState.MANUAL;
    }

    public static void setRobotSystemStateToSemiAuto() {
        currentGameState.robotSystemState = RobotSystemState.SEMI_AUTOMATIC;
    }

    public static void setSystemsToFollowCurrentState() {
        Robot.m_lift.setLiftState(LiftState.FOLLOWING_CURRENT_STATE);
        Robot.m_liftArm.setLiftArmState(LiftArmState.FOLLOWING_CURRENT_STATE);
        Robot.m_elementArm.setElementArmState(ElementArmState.FOLLOWING_CURRENT_STATE);
    }

    public static void sendDebuggingData() {
        updateNextRobotSystemState(); // for debugging purposes.

        SmartDashboard.putString("Debugging/NextState/Cargo Ship Face", nextGameState.cargoShipFace.toString());
        SmartDashboard.putString("Debugging/NextState/Direction", nextGameState.direction.toString());
        SmartDashboard.putString("Debugging/NextState/Field Objective", nextGameState.fieldObjective.toString());
        SmartDashboard.putString("Debugging/NextState/Game Piece", nextGameState.gamePiece.toString());
        SmartDashboard.putString("Debugging/NextState/Robot Action", nextGameState.robotAction.toString());
        SmartDashboard.putString("Debugging/NextState/Robot System State", nextGameState.robotSystemState.toString());
        SmartDashboard.putString("Debugging/NextState/Rocket Placement Height",
                nextGameState.rocketPlacementHeight.toString());
        SmartDashboard.putString("Debugging/NextState/Rocket Side", nextGameState.rocketSide.toString());

        SmartDashboard.putString("Debugging/CurrentState/Cargo Ship Face", currentGameState.cargoShipFace.toString());
        SmartDashboard.putString("Debugging/CurrentState/Direction", currentGameState.direction.toString());
        SmartDashboard.putString("Debugging/CurrentState/Field Objective", currentGameState.fieldObjective.toString());
        SmartDashboard.putString("Debugging/CurrentState/Game Piece", currentGameState.gamePiece.toString());
        SmartDashboard.putString("Debugging/CurrentState/Robot Action", currentGameState.robotAction.toString());
        SmartDashboard.putString("Debugging/CurrentState/Robot System State",
                currentGameState.robotSystemState.toString());
        SmartDashboard.putString("Debugging/CurrentState/Rocket Placement Height",
                currentGameState.rocketPlacementHeight.toString());
        SmartDashboard.putString("Debugging/CurrentState/Rocket Side", currentGameState.rocketSide.toString());
    }

    private static boolean wasRobotSystemStateUpdated() {
        return currentGameState.robotSystemState == lastUpdatedRobotSystemState;
    }

    public static boolean isRobotSystemStateOnTarget() {
        return RobotGameStateManager.wasRobotSystemStateUpdated() && Robot.m_lift.isOnTarget()
                && Robot.m_liftArm.isOnTarget() && Robot.m_elementArm.isOnTarget();
    }

    private static void copyNextToCurrent() {
        currentGameState.robotSystemState = nextGameState.robotSystemState;
        currentGameState.gamePiece = nextGameState.gamePiece;
        currentGameState.fieldObjective = nextGameState.fieldObjective;
        currentGameState.rocketSide = nextGameState.rocketSide;
        currentGameState.cargoShipFace = nextGameState.cargoShipFace;
        currentGameState.direction = nextGameState.direction;
        currentGameState.robotAction = nextGameState.robotAction;
        currentGameState.rocketPlacementHeight = nextGameState.rocketPlacementHeight;
    }

    private static void updateNextRobotSystemState() {
        if (nextGameState.robotAction == RobotAction.FOLDED) {
            if (ControlledGamePieceDetector.getControlledGamePiece() == ControlledGamePiece.HATCH_PANEL)
                nextGameState.robotSystemState = RobotSystemState.FOLDED;
            else {
                nextGameState.robotSystemState = RobotSystemState.FOLDED_WITH_CARGO;
            }

        } else {
            switch (nextGameState.gamePiece) {
            case HATCH_PANEL:
                if (nextGameState.robotAction == RobotAction.FEEDER_COLLECT) {
                    nextGameState.robotSystemState = RobotSystemState.FEEDER_HATCH_START;
                } else {
                    switch (nextGameState.fieldObjective) {
                    case ROCKET:
                        switch (nextGameState.rocketPlacementHeight) {
                        case LEVEL1:
                            nextGameState.robotSystemState = RobotSystemState.L1_HATCH;
                            break;
                        case LEVEL2:
                            nextGameState.robotSystemState = RobotSystemState.L2_HATCH;
                            break;
                        case LEVEL3:
                            nextGameState.robotSystemState = RobotSystemState.L3_HATCH;
                            break;
                        }
                        break;
                    case CARGOSHIP:
                        nextGameState.robotSystemState = RobotSystemState.L1_HATCH;
                        break;
                    }
                }
                break;
            case CARGO:
                if (nextGameState.robotAction == RobotAction.FEEDER_COLLECT) {
                    nextGameState.robotSystemState = getNextArmOrientation() == ArmOrientation.FROM_ABOVE
                            ? RobotSystemState.FEEDER_CARGO_FROM_ABOVE
                            : RobotSystemState.FEEDER_CARGO_FROM_BELOW;
                } else {
                    switch (nextGameState.fieldObjective) {
                    case ROCKET:
                        switch (nextGameState.rocketPlacementHeight) {
                        case LEVEL1:
                            nextGameState.robotSystemState = RobotSystemState.L1_CARGO;
                            break;
                        case LEVEL2:
                            nextGameState.robotSystemState = getNextArmOrientation() == ArmOrientation.FROM_ABOVE
                                    ? RobotSystemState.L2_CARGO_FROM_ABOVE
                                    : RobotSystemState.L2_CARGO_FROM_BELOW;
                            break;
                        case LEVEL3:
                            nextGameState.robotSystemState = RobotSystemState.L3_CARGO;
                            break;
                        }
                        break;
                    case CARGOSHIP:
                        nextGameState.robotSystemState = getNextArmOrientation() == ArmOrientation.FROM_ABOVE
                                ? RobotSystemState.SHIP_CARGO_FROM_ABOVE
                                : RobotSystemState.SHIP_CARGO_FROM_BELOW;
                        break;
                    }
                }
                break;
            }
        }
    }

    public static void updateCurrentConfiguration() {
        updateNextRobotSystemState();
        copyNextToCurrent();
    }

    private static ArmOrientation getNextArmOrientation() {
        if (nextGameState.direction != currentGameState.direction) {
            return ArmOrientation.FROM_BELOW;
        }
        switch (currentGameState.robotSystemState) {
        case FOLDED:
        case L1_HATCH:
        case FEEDER_HATCH_START:
        case FEEDER_HATCH_END:
        case FEEDER_CARGO_FROM_ABOVE:
        case L2_HATCH:
        case L1_CARGO:
        case L2_CARGO_FROM_ABOVE:
        case SHIP_CARGO_FROM_ABOVE:
            return ArmOrientation.FROM_ABOVE;

        case FEEDER_CARGO_FROM_BELOW:
        case L3_HATCH:
        case L2_CARGO_FROM_BELOW:
        case L3_CARGO:
        case SHIP_CARGO_FROM_BELOW:
            return ArmOrientation.FROM_BELOW;
        default:
            DriverStation.reportWarning(
                    "getNextArmOrientation reached Default Case with " + currentGameState.robotSystemState.toString(),
                    true);
            return ArmOrientation.FROM_ABOVE; // This is the default state
        }
    }
}

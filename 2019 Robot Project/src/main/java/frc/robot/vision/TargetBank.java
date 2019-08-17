/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import frc.robot.RobotGameState;
import frc.robot.RobotGameStateManager;
import frc.robot.RobotGameState.RobotAction;

/**
 * This class holds all the targets in the field and their angles. It serves to
 * find a specific target according to estimated angles.
 */
public class TargetBank {

    /**
     * Contains all the targets and their angles.
     */
    public enum AlignmentTarget {

        RIGHT_CARGOSHIP(90.0), LEFT_CARGOSHIP(-90.0), LEFT_ROCKET_CARGO(90), RIGHT_ROCKET_CARGO(-90),
        CLOSE_LEFT_ROCKET(28.75), FAR_LEFT_ROCKET(151.25), CLOSE_RIGHT_ROCKET(-28.75), FAR_RIGHT_ROCKET(-151.25),
        CARGO_SHIP_FRONT(0), FEEDER(180);

        private final double absoluteAngle;

        AlignmentTarget(double absoluteAngle) {
            this.absoluteAngle = absoluteAngle;
        }

        /**
         * @return the angle for the specific target.
         */
        public double getAbsoluteAngle() {
            return absoluteAngle;
        }
    }

    /**
     * @param estimatedAngle the estimated angle from the robot to the target.
     * @return the angle of the target that is closest to the estimated angle.
     */
    public static AlignmentTarget getAlignmentTarget(double estimatedAngle) {
        if (RobotGameStateManager.currentGameState.robotAction == RobotAction.FEEDER_COLLECT) {
            return AlignmentTarget.FEEDER;
        } else {
            switch (RobotGameStateManager.nextGameState.fieldObjective) {
            case CARGOSHIP:
                if (RobotGameStateManager.nextGameState.cargoShipFace == RobotGameState.CargoShipFace.FRONT) {
                    return AlignmentTarget.CARGO_SHIP_FRONT;
                } else {
                    if (estimatedAngle > 0) {
                        return AlignmentTarget.RIGHT_CARGOSHIP;
                    } else {
                        return AlignmentTarget.LEFT_CARGOSHIP;
                    }
                }
            case ROCKET:
            default:
                if (RobotGameStateManager.nextGameState.gamePiece == RobotGameState.GamePiece.CARGO) { // Cargo Game Piece
                    if (estimatedAngle > 0) {
                        return AlignmentTarget.LEFT_ROCKET_CARGO;
                    } else {
                        return AlignmentTarget.RIGHT_ROCKET_CARGO;
                    }
                } else { // Hatch Panel Game Piece
                    if (RobotGameStateManager.nextGameState.rocketSide == RobotGameState.RocketSide.RIGHT) { // Right
                                                                                                             // Rocket
                        if (estimatedAngle > -90 && estimatedAngle < 90) {
                            return AlignmentTarget.CLOSE_RIGHT_ROCKET;
                        } else {
                            return AlignmentTarget.FAR_RIGHT_ROCKET;
                        }
                    } else { // Left Rocket
                        if (estimatedAngle > -90 && estimatedAngle < 90) {
                            return AlignmentTarget.CLOSE_LEFT_ROCKET;
                        } else {
                            return AlignmentTarget.FAR_LEFT_ROCKET;
                        }
                    }
                }
            }
        }
    }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

/**
 * Add your docs here.
 */
public class RobotGameState {
    public RobotSystemState robotSystemState = RobotSystemState.FOLDED;
    public GamePiece gamePiece = GamePiece.HATCH_PANEL;
    public FieldObjective fieldObjective = FieldObjective.ROCKET;
    public RocketSide rocketSide = RocketSide.LEFT;
    public CargoShipFace cargoShipFace = CargoShipFace.FRONT;
    public Direction direction = Direction.FORWARD;
    public RocketPlacementHeight rocketPlacementHeight = RocketPlacementHeight.LEVEL1;
    public RobotAction robotAction = RobotAction.FOLDED;

    public enum RobotSystemState {
        SEMI_AUTOMATIC(null), //
        FOLDED(ROBOT_PROFILE.robotStateParams.folded), //
        FOLDED_WITH_CARGO(ROBOT_PROFILE.robotStateParams.foldedWithCargo), //
        CLIMB_LEVEL3(ROBOT_PROFILE.robotStateParams.climbLevel3), //
        CLIMB_LEVEL2(ROBOT_PROFILE.robotStateParams.climbLevel2), //
        FRONT_INTAKE_ARM_GAP_LOW(ROBOT_PROFILE.robotStateParams.frontIntakeArmGapLow), //
        FRONT_INTAKE_ARM_GAP_HIGH(ROBOT_PROFILE.robotStateParams.frontIntakeArmGapHigh), // Requires special logic,
                                                                                         // depends on
        // your current state
        L1_HATCH(ROBOT_PROFILE.robotStateParams.l1Hatch), //
        FEEDER_HATCH_START(ROBOT_PROFILE.robotStateParams.feederHatchStart), //
        FEEDER_HATCH_END(ROBOT_PROFILE.robotStateParams.feederHatchEnd), //
        FEEDER_CARGO_FROM_ABOVE(ROBOT_PROFILE.robotStateParams.feederCargoFromAbove), //
        FEEDER_CARGO_FROM_BELOW(ROBOT_PROFILE.robotStateParams.feederCargoFromBelow), //
        FLOOR_CARGO(ROBOT_PROFILE.robotStateParams.floorCargo), //
        FLOOR_HATCH_START(ROBOT_PROFILE.robotStateParams.floorHatchStart), //
        FLOOR_HATCH_DELIVERY(ROBOT_PROFILE.robotStateParams.floorHatchDelivery), //
        FLOOR_HATCH_END(ROBOT_PROFILE.robotStateParams.floorHatchEnd), //
        L2_HATCH(ROBOT_PROFILE.robotStateParams.l2Hatch), //
        L2_HATCH_FOLDED(ROBOT_PROFILE.robotStateParams.l2HatchFolded), //
        L3_HATCH(ROBOT_PROFILE.robotStateParams.l3Hatch), //
        L1_CARGO(ROBOT_PROFILE.robotStateParams.l1Cargo), //
        L2_CARGO_FROM_ABOVE(ROBOT_PROFILE.robotStateParams.l2CargoFromAbove), //
        L2_CARGO_FROM_BELOW(ROBOT_PROFILE.robotStateParams.l2CargoFromBelow), //
        L3_CARGO(ROBOT_PROFILE.robotStateParams.l3Cargo), //
        SHIP_CARGO_FROM_ABOVE(ROBOT_PROFILE.robotStateParams.shipCargoFromAbove), //
        SHIP_CARGO_FROM_BELOW(ROBOT_PROFILE.robotStateParams.shipCargoFromBelow), //
        ABORT_CARGO_FLOOR_CARGO(ROBOT_PROFILE.robotStateParams.abortCargoFloorCollect);

        public RobotConfiguration robotConfiguration;

        private RobotSystemState(RobotConfiguration robotConfiguration) {
            this.robotConfiguration = robotConfiguration;
        }
    }

    public enum Direction {
        FORWARD, BACKWARD;
    }

    public enum GamePiece {
        CARGO, HATCH_PANEL;
    }

    public enum FieldObjective {
        ROCKET, CARGOSHIP;
    }

    public enum RocketPlacementHeight {
        LEVEL1, LEVEL2, LEVEL3;
    }

    public enum RobotAction {
        FLOOR_COLLECT, FEEDER_COLLECT, CLIMB, PRE_PLACEMENT, POST_PLACEMENT, FOLDED, ABORT_CARGO_FLOOR_COLLECT;
    }

    // VISION:
    public enum RocketSide {
        LEFT, RIGHT;
    }

    public enum CargoShipFace {
        FRONT, SIDE;
    }
}

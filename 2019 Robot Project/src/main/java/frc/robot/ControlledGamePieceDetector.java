/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.bumblelib.util.hardware.Sharp_GP2Y0A41SK0F;
import frc.robot.subsystems.HatchHandler.SolenoidState;

/**
 * Add your docs here.
 */
public class ControlledGamePieceDetector {

    private static final double CONTROLLED_CARGO_DISTANCE = 18;
    private static final double CONTROLLED_HATCH_PANEL_DISTANCE_MIN = 26;
    private static final double CONTROLLED_HATCH_PANEL_DISTANCE_MAX = 42;

    private static final double HALF_CONTROLLED_CARGO_DISTANCE = 10.5;

    private static final double CARGO_IN_DISTANCE = 10.0;

    private static int SWITCH_GAME_PIECE_THRESHOLD = 10;

    private static Sharp_GP2Y0A41SK0F IR_Sensor = new Sharp_GP2Y0A41SK0F(RobotMap.ElementArmPorts.DISTANCE_SENSOR);
    private static ControlledGamePiece controlledGamePiece = ControlledGamePiece.NONE;
    private static int noneCounter = 0, cargoCounter = 0, hatchCounter = 0;

    public enum ControlledGamePiece {
        NONE, CARGO, HATCH_PANEL;
    }

    public static void update() {
        double distance = IR_Sensor.getDistance();
        if (distance <= CONTROLLED_CARGO_DISTANCE) {
            noneCounter = 0;
            cargoCounter++;
            hatchCounter = 0;
        } else if (CONTROLLED_HATCH_PANEL_DISTANCE_MIN <= distance && distance <= CONTROLLED_HATCH_PANEL_DISTANCE_MAX) {
            noneCounter = 0;
            cargoCounter = 0;
            hatchCounter++;
        } else {
            noneCounter++;
            cargoCounter = 0;
            hatchCounter = 0;
        }

        if (cargoCounter >= SWITCH_GAME_PIECE_THRESHOLD) {
            controlledGamePiece = ControlledGamePiece.CARGO;
        } else if (hatchCounter >= SWITCH_GAME_PIECE_THRESHOLD) {
            controlledGamePiece = ControlledGamePiece.HATCH_PANEL;
        } else if (noneCounter >= SWITCH_GAME_PIECE_THRESHOLD) {
            controlledGamePiece = ControlledGamePiece.NONE;
        }
    }

    public static ControlledGamePiece getControlledGamePiece() {
        if (Robot.m_hatchHandler.getHoldState() == SolenoidState.EXTEND) {
            return ControlledGamePiece.HATCH_PANEL;
        } else if (controlledGamePiece == ControlledGamePiece.CARGO) {
            return ControlledGamePiece.CARGO;
        } else {
            return ControlledGamePiece.NONE;
        }
    }

    public static boolean isCargoIn() {
        return IR_Sensor.getDistance() <= CARGO_IN_DISTANCE;
    }

    public static boolean isHalfControllingCargo() {
        return controlledGamePiece == ControlledGamePiece.CARGO
                && IR_Sensor.getDistance() >= HALF_CONTROLLED_CARGO_DISTANCE;
    }

    public static void sendDebuggingData() {
        SmartDashboard.putNumber("Debugging/ControlledGamePiece/IR Distance", IR_Sensor.getDistance());
        SmartDashboard.putString("Debugging/ControlledGamePiece/Detected Game Piece",
                getControlledGamePiece().toString());
    }
}

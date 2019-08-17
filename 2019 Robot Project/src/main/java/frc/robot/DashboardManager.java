/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotGameState.CargoShipFace;
import frc.robot.RobotGameState.FieldObjective;
import frc.robot.RobotGameState.GamePiece;
import frc.robot.RobotGameState.RocketPlacementHeight;
import frc.robot.RobotGameState.RocketSide;

/**
 * Add your docs here.
 */
public class DashboardManager {

    public static void updateSmartDashboard() {

        // General.
        SmartDashboard.putString("Game/Action", RobotGameStateManager.currentGameState.robotAction.toString());
        SmartDashboard.putString("Game/Climb Mode", RobotGameStateManager.climbMode.toString());

        // Game Piece.
        SmartDashboard.putString("Game/ ", RobotGameStateManager.nextGameState.gamePiece.toString());
        SmartDashboard.putBoolean("Game/Game Piece",
                RobotGameStateManager.nextGameState.gamePiece == GamePiece.HATCH_PANEL);

        // Rocket.
        SmartDashboard.putBoolean("Game/Level 3",
                RobotGameStateManager.nextGameState.rocketPlacementHeight == RocketPlacementHeight.LEVEL3);
        SmartDashboard.putBoolean("Game/Level 2",
                RobotGameStateManager.nextGameState.rocketPlacementHeight == RocketPlacementHeight.LEVEL2);
        SmartDashboard.putBoolean("Game/Level 1",
                RobotGameStateManager.nextGameState.rocketPlacementHeight == RocketPlacementHeight.LEVEL1);

        SmartDashboard.putBoolean("Game/Right", RobotGameStateManager.nextGameState.rocketSide == RocketSide.RIGHT
                && RobotGameStateManager.nextGameState.fieldObjective == FieldObjective.ROCKET);
        SmartDashboard.putBoolean("Game/Left", RobotGameStateManager.nextGameState.rocketSide == RocketSide.LEFT
                && RobotGameStateManager.nextGameState.fieldObjective == FieldObjective.ROCKET);

        // Ship.
        SmartDashboard.putBoolean("Game/Side", RobotGameStateManager.nextGameState.cargoShipFace == CargoShipFace.SIDE);
        SmartDashboard.putBoolean("Game/Front",
                RobotGameStateManager.nextGameState.cargoShipFace == CargoShipFace.FRONT);

        SmartDashboard.putBoolean("Game/Cargoship",
                RobotGameStateManager.nextGameState.fieldObjective == FieldObjective.CARGOSHIP);

        SmartDashboard.putString("Game/IntakeArm Mode",
                Robot.m_intakeArm.getCurrentIntakeArmState().toDashboardString());
        SmartDashboard.putString("Game/LiftArm Mode", Robot.m_liftArm.getCurrentLiftArmState().toDashboardString());
        SmartDashboard.putString("Game/ElementArm Mode",
                Robot.m_elementArm.getCurrentElementArmState().toDashboardString());
        SmartDashboard.putString("Game/Lift Mode", Robot.m_lift.getCurrentLiftState().toDashboardString());

        // Camera
        SmartDashboard.putString("Game/Direction", RobotGameStateManager.nextGameState.direction.toString());
        SmartDashboard.putString("Game/Gear", Robot.m_drivetrain.getGear().toString());

        SmartDashboard.putNumber("Game/Gyro", Robot.m_drivetrain.getGyro().getYaw());
    }
}

package frc.robot.autonomous.paths;

import frc.bumblelib.bumblelib_autonomous.pathing.path.SingularPath;
import frc.bumblelib.bumblelib_autonomous.pathing.path.SymmetricPath;
import frc.robot.autonomous.paths.calibration_paths.CurveCalibrationPath;
import frc.robot.autonomous.paths.calibration_paths.LongCalibrationPath;
import frc.robot.autonomous.paths.calibration_paths.ShortCalibrationPath;

public class Paths {

    // Calibration.
    public static SingularPath shortCalibrationPath, longCalibrationPath, curvePath;

    public static SingularPath relativeToCloseRocket;
    public static SymmetricPath feederToFarRocket, closeRocketToFeeder, midPlatformToFrontShipPlacement,
            frontShipPlacementToFeeder, feederToCloseShipPlacement;

    public Paths() {

        // Calibration.
        shortCalibrationPath = new ShortCalibrationPath();
        longCalibrationPath = new LongCalibrationPath();
        curvePath = new CurveCalibrationPath();

        relativeToCloseRocket = new RelativeToCloseRocket();
        feederToFarRocket = new FeederToFarRocket();
        closeRocketToFeeder = new CloseRocketToFeeder();
        midPlatformToFrontShipPlacement = new MidPlatformToFrontShipPlacement();
        frontShipPlacementToFeeder = new FrontShipPlacementToFeeder();
        feederToCloseShipPlacement = new FeederToCloseShipPlacement();
    }
}

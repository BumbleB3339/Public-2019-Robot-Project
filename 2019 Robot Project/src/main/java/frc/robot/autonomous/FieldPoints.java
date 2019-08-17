package frc.robot.autonomous;

import static frc.robot.profiles.InitProfiles.FIELD_PROFILE;

import frc.bumblelib.bumblelib_autonomous.pathing.field_point.SingularFieldPoint;
import frc.bumblelib.bumblelib_autonomous.pathing.field_point.SymmetricFieldPoint;

public class FieldPoints {

    // CALIBRATION POINTS - DO NOT TOUCH
    public static SingularFieldPoint zeroPoint, shortCalibrationEnd, longCalibrationEnd, curveCalibrationEnd;

    // General points.
    public static SymmetricFieldPoint level1OuterCorner, feederCenter, level23ConnectionOnLevel1Edge;

    // Rocket.
    public static SymmetricFieldPoint farRocketFaceCenter, closeRocketFaceCenter, midRocketFaceCenter;

    // CargoShip
    public static SymmetricFieldPoint frontShipPlacement, sideShipPlacementClose, sideShipPlacementMid, sideShipPlacementFar;

    public FieldPoints() {
        shortCalibrationEnd = new SingularFieldPoint();
        shortCalibrationEnd.addX(0.0);
        shortCalibrationEnd.addY(3.0);

        longCalibrationEnd = new SingularFieldPoint();
        longCalibrationEnd.addX(0.0);
        longCalibrationEnd.addY(5.0);

        curveCalibrationEnd = new SingularFieldPoint();
        curveCalibrationEnd.addX(1.0);
        curveCalibrationEnd.addY(3.0);

        zeroPoint = new SingularFieldPoint();
        zeroPoint.addX(0.0);
        zeroPoint.addY(0.0);

        level1OuterCorner = new SymmetricFieldPoint();
        level1OuterCorner.addX(FIELD_PROFILE.sideWallToLevel1FlatEdge());
        level1OuterCorner.addY(FIELD_PROFILE.allianceWallToLevel1FlatEdge());

        closeRocketFaceCenter = new SymmetricFieldPoint();
        closeRocketFaceCenter.addX(FIELD_PROFILE.sideWallToRocket())
                             .addX(FIELD_PROFILE.rocketDepth().multiplyBy(0.5));
        closeRocketFaceCenter.addY(FIELD_PROFILE.allianceWallToCloseRocketCorner())
                             .addY((FIELD_PROFILE.rocketDepth().multiplyBy(1.0 / Math.tan(FIELD_PROFILE.absoluteRocketAngle()))).multiplyBy(0.5));

        farRocketFaceCenter = new SymmetricFieldPoint();
        farRocketFaceCenter.addX(FIELD_PROFILE.sideWallToRocket())
                           .addX(FIELD_PROFILE.rocketDepth().multiplyBy(0.5));
        farRocketFaceCenter.addY(FIELD_PROFILE.allianceWallToCloseRocketCorner())
                           .addY(FIELD_PROFILE.rocketLength())
                           .subtractY((FIELD_PROFILE.rocketDepth().multiplyBy(1.0 / Math.tan(FIELD_PROFILE.absoluteRocketAngle()))).multiplyBy(0.5));                

        feederCenter = new SymmetricFieldPoint();
        feederCenter.addX(FIELD_PROFILE.sideWallToMidFeeder());
        feederCenter.addY(0.0);

        midRocketFaceCenter = new SymmetricFieldPoint();
        midRocketFaceCenter.addX(FIELD_PROFILE.sideWallToRocket())
                           .addX(FIELD_PROFILE.rocketDepth());
        midRocketFaceCenter.addY(FIELD_PROFILE.allianceWallToCloseRocketCorner())
                           .addY(FIELD_PROFILE.rocketLength().multiplyBy(0.5));
        
        frontShipPlacement = new SymmetricFieldPoint();
        frontShipPlacement.addX(FIELD_PROFILE.sideWallToCargoshipSide())
                          .addX(FIELD_PROFILE.cargoShipSideToFrontHatch());
        frontShipPlacement.addY(FIELD_PROFILE.allianceWallToLevel1FlatEdge())
                          .addY(FIELD_PROFILE.level1FlatEdgeToFrontCargoShipFace());
        
        sideShipPlacementClose = new SymmetricFieldPoint();
        sideShipPlacementClose.addX(FIELD_PROFILE.sideWallToCargoshipSide());
        sideShipPlacementClose.addY(FIELD_PROFILE.allianceWallToLevel1FlatEdge())
                              .addY(FIELD_PROFILE.level1FlatEdgeToFrontCargoShipFace())
                              .addY(FIELD_PROFILE.frontShipFaceToCloseHatchPlacement());

        sideShipPlacementMid = new SymmetricFieldPoint();
        sideShipPlacementMid.addX(FIELD_PROFILE.sideWallToCargoshipSide());
        sideShipPlacementMid.addY(FIELD_PROFILE.allianceWallToLevel1FlatEdge())
                            .addY(FIELD_PROFILE.level1FlatEdgeToFrontCargoShipFace())
                            .addY(FIELD_PROFILE.frontShipFaceToMidHatchPlacement());

        sideShipPlacementFar = new SymmetricFieldPoint();
        sideShipPlacementFar.addX(FIELD_PROFILE.sideWallToCargoshipSide());
        sideShipPlacementFar.addY(FIELD_PROFILE.allianceWallToLevel1FlatEdge())
                            .addY(FIELD_PROFILE.level1FlatEdgeToFrontCargoShipFace())
                            .addY(FIELD_PROFILE.frontShipFaceToFarHatchPlacement());

        level23ConnectionOnLevel1Edge = new SymmetricFieldPoint();
        level23ConnectionOnLevel1Edge.addX(FIELD_PROFILE.sideWallToLevel3Edge());
        level23ConnectionOnLevel1Edge.addY(FIELD_PROFILE.allianceWallToLevel1FlatEdge());
    }
}

package frc.robot.autonomous.paths;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Direction;
import frc.bumblelib.bumblelib_autonomous.pathing.path.SymmetricPath;
import frc.bumblelib.bumblelib_autonomous.pathing.path_point.SymmetricPathPoint;
import frc.robot.autonomous.FieldPoints;
import frc.robot.profiles.InitProfiles;
import frc.robot.subsystems.Drivetrain.Gear;
import jaci.pathfinder.Pathfinder;

public class FrontShipPlacementToFeeder extends SymmetricPath {

    private static final double MAX_VEL = 2.5;
    private static final double FEEDER_PRE_VISION_Y_DISPLACEMENT = 2.0;
    private static final double FEEDER_PRE_VISION_X_DISPLACEMENT = 0.4;
    private static final double FEEDER_PRE_VISION_ANGLE = -15.0;

    public FrontShipPlacementToFeeder() {
        super("FrontShipPlacementToFeeder", Gear.SPEED_GEAR, MAX_VEL, Direction.REVERSE,
                ROBOT_PROFILE.pathfinderParams.speedGear.fastPathPreset);

        add(new SymmetricPathPoint(FieldPoints.frontShipPlacement, 0.0,
                InitProfiles.ROBOT_PROFILE.robotReferencePoints.frontCenter, false));

        add(new SymmetricPathPoint(FieldPoints.feederCenter.displace(-FEEDER_PRE_VISION_X_DISPLACEMENT, FEEDER_PRE_VISION_Y_DISPLACEMENT),
                FEEDER_PRE_VISION_ANGLE, InitProfiles.ROBOT_PROFILE.robotReferencePoints.jaciCenterOfRotation, false));

        add(new SymmetricPathPoint(
                FieldPoints.feederCenter.displace(-FEEDER_PRE_VISION_X_DISPLACEMENT, FEEDER_PRE_VISION_Y_DISPLACEMENT).displaceByAngle(0.5, Pathfinder.boundHalfDegrees(FEEDER_PRE_VISION_ANGLE + 180.0)),
                FEEDER_PRE_VISION_ANGLE, InitProfiles.ROBOT_PROFILE.robotReferencePoints.jaciCenterOfRotation, false));
    }
}
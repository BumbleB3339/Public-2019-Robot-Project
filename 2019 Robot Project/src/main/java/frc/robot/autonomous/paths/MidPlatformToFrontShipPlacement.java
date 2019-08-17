package frc.robot.autonomous.paths;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Direction;
import frc.bumblelib.bumblelib_autonomous.pathing.path.SymmetricPath;
import frc.bumblelib.bumblelib_autonomous.pathing.path_point.SymmetricPathPoint;
import frc.robot.autonomous.FieldPoints;
import frc.robot.autonomous.InitialPathPoints;
import frc.robot.profiles.InitProfiles;
import frc.robot.subsystems.Drivetrain.Gear;

public class MidPlatformToFrontShipPlacement extends SymmetricPath {

    private static final double MAX_VEL = 2.5;

    public MidPlatformToFrontShipPlacement() {
        super("MidPlatformToFrontShipPlacement", Gear.SPEED_GEAR, MAX_VEL, Direction.FORWARD,
                ROBOT_PROFILE.pathfinderParams.speedGear.fastPathPreset);

        add(InitialPathPoints.level1Inner);

        add(new SymmetricPathPoint(FieldPoints.frontShipPlacement.displace(0.0, -0.5), 0.0,
                InitProfiles.ROBOT_PROFILE.robotReferencePoints.frontCenter, false));
        add(new SymmetricPathPoint(FieldPoints.frontShipPlacement.displace(0.0, -0.5).displaceByAngle(0.02, 0.0), 0.0,
                InitProfiles.ROBOT_PROFILE.robotReferencePoints.frontCenter, false));
    }
}
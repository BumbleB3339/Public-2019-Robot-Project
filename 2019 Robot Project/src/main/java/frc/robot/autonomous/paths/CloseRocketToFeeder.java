package frc.robot.autonomous.paths;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Direction;
import frc.bumblelib.bumblelib_autonomous.pathing.path.SymmetricPath;
import frc.bumblelib.bumblelib_autonomous.pathing.path_point.SymmetricPathPoint;
import frc.robot.autonomous.FieldPoints;
import frc.robot.profiles.InitProfiles;
import frc.robot.subsystems.Drivetrain.Gear;
import jaci.pathfinder.Pathfinder;

public class CloseRocketToFeeder extends SymmetricPath {

    private static final double MAX_VEL = 1.7;
    
    private static final double CLOSE_ROCKET_PERPENDICULAR_ANGLE = -InitProfiles.FIELD_PROFILE.absoluteRocketAngle() - 90;
    private static final double CLOSE_ROCKET_PLACEMENT_ANGLE = Pathfinder.boundHalfDegrees(CLOSE_ROCKET_PERPENDICULAR_ANGLE + 180);
    private static final double CLOSE_ROCKET_PLACEMENT_DISTANCE = 0.0;

    private static final double FEEDER_COLLECTION_DISTANCE = 0.0;

    public CloseRocketToFeeder() {
        super("CloseRocketToFeeder", Gear.SPEED_GEAR, MAX_VEL, Direction.REVERSE,
                ROBOT_PROFILE.pathfinderParams.speedGear.fastPathPreset);

        add(new SymmetricPathPoint(FieldPoints.closeRocketFaceCenter.displaceByAngle(CLOSE_ROCKET_PLACEMENT_DISTANCE, CLOSE_ROCKET_PERPENDICULAR_ANGLE), 
            CLOSE_ROCKET_PLACEMENT_ANGLE, InitProfiles.ROBOT_PROFILE.robotReferencePoints.frontCenter, false));

        add(new SymmetricPathPoint(FieldPoints.feederCenter.displace(0.0, FEEDER_COLLECTION_DISTANCE + 2.0), 
            -10.0, InitProfiles.ROBOT_PROFILE.robotReferencePoints.realCenterOfRotation, false));
        
        add(new SymmetricPathPoint(FieldPoints.feederCenter.displace(0.0, FEEDER_COLLECTION_DISTANCE + 2.0).displaceByAngle(-2.0, -10.0), 
           -10.0, InitProfiles.ROBOT_PROFILE.robotReferencePoints.realCenterOfRotation, false));
    }
}
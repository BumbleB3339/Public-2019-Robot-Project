package frc.robot.autonomous.paths;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Direction;
import frc.bumblelib.bumblelib_autonomous.pathing.path.SymmetricPath;
import frc.bumblelib.bumblelib_autonomous.pathing.path_point.SymmetricPathPoint;
import frc.robot.autonomous.FieldPoints;
import frc.robot.profiles.InitProfiles;
import frc.robot.subsystems.Drivetrain.Gear;

public class FeederToFarRocket extends SymmetricPath {

    private static final double MAX_VEL = 2.5;

    private static final double FAR_ROCKET_PERPENDICULAR_ANGLE = InitProfiles.FIELD_PROFILE.farRocketAngle() - 180.0;
    private static final double FAR_ROCKET_PRE_PLACEMENT_ANGLE = -30.0;
    private static final double FAR_ROCKET_PRE_PLACEMENT_DISTANCE = 1.35;
     
    private static final double FEEDER_COLLECTION_ANGLE = 0.0;
    private static final double FEEDER_COLLECTION_DISTANCE = 0.0;

    private static final double MID_POINT_ROCKET_DISTANCE = 0.7;
    private static final double MID_POINT_ANGLE = 18.0;

    public FeederToFarRocket() {
        super("FeederToFarRocket", Gear.SPEED_GEAR, MAX_VEL, Direction.FORWARD,
                ROBOT_PROFILE.pathfinderParams.speedGear.fastPathPreset);

        add(new SymmetricPathPoint(FieldPoints.feederCenter.displace(0.0, FEEDER_COLLECTION_DISTANCE), 
            FEEDER_COLLECTION_ANGLE, InitProfiles.ROBOT_PROFILE.robotReferencePoints.rearCenter, false));
        
        add(new SymmetricPathPoint(FieldPoints.midRocketFaceCenter.displace(-MID_POINT_ROCKET_DISTANCE, 0.0), 
            MID_POINT_ANGLE , ROBOT_PROFILE.robotReferencePoints.realCenterOfRotation, false));

        // add(new SymmetricPathPoint(FieldPoints.midRocketFaceCenter.displace(-MID_POINT_ROCKET_DISTANCE + 0.25, 1.3), 
        //     23.0 , ROBOT_PROFILE.robotReferencePoints.rearCenter, false));
        
        add(new SymmetricPathPoint(FieldPoints.farRocketFaceCenter.displaceByAngle(FAR_ROCKET_PRE_PLACEMENT_DISTANCE, FAR_ROCKET_PERPENDICULAR_ANGLE).displaceByAngle(0.0, FAR_ROCKET_PERPENDICULAR_ANGLE - 90.0),
        FAR_ROCKET_PRE_PLACEMENT_ANGLE, InitProfiles.ROBOT_PROFILE.robotReferencePoints.realCenterOfRotation, false));
    }
}
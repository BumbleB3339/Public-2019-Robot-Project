package frc.robot.autonomous.paths;

import static frc.robot.profiles.InitProfiles.ROBOT_PROFILE;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Direction;
import frc.bumblelib.bumblelib_autonomous.pathing.path.SingularPath;
import frc.bumblelib.bumblelib_autonomous.pathing.path_point.SingularPathPoint;
import frc.robot.autonomous.FieldPoints;
import frc.robot.subsystems.Drivetrain.Gear;;

public class RelativeToCloseRocket extends SingularPath {

    private static final double MAX_VEL = 2.0;

    public RelativeToCloseRocket() {
        super("RelativeCloseRocket", Gear.SPEED_GEAR, MAX_VEL, Direction.FORWARD,
                ROBOT_PROFILE.pathfinderParams.speedGear.fastPathPreset);
        add(new SingularPathPoint(FieldPoints.zeroPoint, 0.0, ROBOT_PROFILE.robotReferencePoints.realCenterOfRotation));
        add(new SingularPathPoint(FieldPoints.zeroPoint.displace(0.0, 3.5), 0.0,
                ROBOT_PROFILE.robotReferencePoints.realCenterOfRotation));
        setRelative();
    }
}
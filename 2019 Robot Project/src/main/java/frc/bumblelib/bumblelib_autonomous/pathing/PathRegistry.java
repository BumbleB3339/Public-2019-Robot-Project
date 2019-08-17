package frc.bumblelib.bumblelib_autonomous.pathing;

import java.util.ArrayList;

import frc.bumblelib.bumblelib_autonomous.CSVManager;
import frc.bumblelib.bumblelib_autonomous.CSVSendable;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.bumblelib.bumblelib_autonomous.pathing.path.SingularPath;
import frc.bumblelib.bumblelib_autonomous.pathing.rotation.Rotation;
import frc.bumblelib.bumblelib_autonomous.pathing.rotation.SingularAngle;
import frc.robot.subsystems.Drivetrain.Gear;

public class PathRegistry {

    private static PathRegistry registryInstance = null;

    private ArrayList<CSVSendable> paths = new ArrayList<>();

    private PathRegistry() {
    }

    public static PathRegistry getInstance() {
        if (registryInstance == null) {
            registryInstance = new PathRegistry();
        }
        return registryInstance;
    }

    /**
     * Add a path to the registry.
     * 
     * @param path The path to add.
     */
    public void add(CSVSendable path) {
        paths.add(path);
    }

    /**
     * Generates all paths in the registry. This should be called during robot
     * initialization.
     */
    public void generatePaths() {
        CSVManager.clearLatestFolder(); // Clear the latest folder first.
        CSVManager.validateFolders(); // Validate folder existance

        if (!CSVManager.doesSendableExist(new Rotation(Gear.POWER_GEAR, new SingularAngle(180), true), Alliance.RED,
        Side.RIGHT)) { // Write more generaly (check other files as well)
            for (int i = 0; i <= 180; i++) {
                CSVManager.validateTrajectory(new Rotation(Gear.POWER_GEAR, new SingularAngle(i), true), Alliance.RED,
                        Side.RIGHT);
            }
        } 
        //CSVManager.validateTrajectory(new Rotation(Gear.POWER_GEAR, new SingularAngle(74), true), Alliance.RED, Side.LEFT);


        for (CSVSendable path : paths) {
            CSVManager.validateTrajectory(path, Alliance.RED, Side.RIGHT);
            CSVManager.validateTrajectory(path, Alliance.BLUE, Side.RIGHT);

            if (!(path instanceof SingularPath || path instanceof SingularAngle)) { // No need to generate other side if
                                                                                    // singular.
                CSVManager.validateTrajectory(path, Alliance.RED, Side.LEFT);
                CSVManager.validateTrajectory(path, Alliance.BLUE, Side.LEFT);
            }
        }
    }
}

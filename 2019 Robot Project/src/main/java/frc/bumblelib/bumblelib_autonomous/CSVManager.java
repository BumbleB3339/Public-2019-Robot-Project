package frc.bumblelib.bumblelib_autonomous;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.bumblelib.bumblelib_autonomous.pathing.path.SymmetricPath;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class CSVManager {

    private static final String LATEST = "/home/lvuser/paths/latest_paths/";
    private static final String REPOSITORY = "/home/lvuser/paths/path_repository/";


	/**
	 * Checks if the Trajectory of the given path with the given alliance and side parameters exists in the path_repository folder. 
	 * If it exists, it is copied to the latest_paths folder. If it doesn't exist, it is generated and then copied.
	 * 
	 * @param path
	 * 		The path to check.
	 * @param alliance
	 * 		The alliance for which to check.
	 * @param side
	 * 		The side for which to check.
	 */
	public static File validateTrajectory(CSVSendable path, Alliance alliance, Side side) {
		File latestFile = new File(LATEST + path.getNameTemplate(alliance, side));
		File repoFile = new File(REPOSITORY + generateHash(path.getDescription(alliance, side)) + ".csv");

		if (!repoFile.exists()) { // Create path and write to file in repo.
			// Report path generation status to Driver Station.
			String dsWarning = path instanceof SymmetricPath ? "Path '" + path.getName() + "'" + ", Alliance: " + alliance + ", Side: " + side :
			"Path '" + path.getName() + "'" + ", Alliance: " + alliance;	
			DriverStation.reportWarning(dsWarning + " was not found, generating CSV... HASH: " + generateHash(path.getDescription(alliance, side)), false);
			double startTime = Timer.getFPGATimestamp();

			Trajectory trajectory = path.generateTrajectory(alliance, side);
			try {
				repoFile.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}
			Pathfinder.writeToCSV(repoFile, trajectory);

			// Calculate generation time and print message.
			double genTime = Timer.getFPGATimestamp() - startTime;
			DriverStation.reportWarning(dsWarning + " generated in " + genTime + " seconds", false);

		} else {
			// Report path generation status to Driver Station.
			String dsWarning = path instanceof SymmetricPath ? "Path '" + path.getName() + "'" + ", Alliance: " + alliance + ", Side: " + side + " already generated" :
			"Path '" + path.getName() + "'" + ", Alliance: " + alliance + " already generated";
			DriverStation.reportWarning(dsWarning, false);
		}

		try { // Copy repo file to latest folder.
			Files.copy(repoFile.toPath(), latestFile.toPath(), StandardCopyOption.REPLACE_EXISTING);
		} catch (IOException e) {
			e.printStackTrace();
		}

		return latestFile;
	}

	/**
	 * Checks if a trajectory CSV already exists on RoboRIO.
	 * @param sendable
	 * @param alliance
	 * @param side
	 * @return
	 */
	public static boolean doesSendableExist(CSVSendable sendable, Alliance alliance, Side side) {
		File repoFile = new File(REPOSITORY + generateHash(sendable.getDescription(alliance, side)) + ".csv");
		return repoFile.exists();
	}

	/**
	 * Returns the trajectory of the given path with respect to given alliance and side parameters.
	 * 
	 * @param path
	 * 		The path to generate the trajectory from.
	 * @param alliance
	 * 		The alliance of the given path.
	 * @param side
	 * 		The side of the given path.
	 * @return the trajectory of the given path with respect to alliance and side parameters.
	 */
    public static Trajectory getTrajectory(CSVSendable path, Alliance alliance, Side side) {
		// Make sure that the file exists and read trajectory from CSV.
		File latestFile = validateTrajectory(path, alliance, side);
		try {
			return Pathfinder.readFromCSV(latestFile);
		} catch (IOException e) {
			e.printStackTrace();
			return null;
		}
	}
	
	/**
	 * Deletes the contents of the 'latest_paths' folder
	 */
	public static void clearLatestFolder() {
		File dir = new File(LATEST);
		if (dir.exists()) {
			for (File file : dir.listFiles()) {
				file.delete();
			}
		}
	}

	public static void validateFolders() {
		File latest = new File(LATEST + "rotations/");
		if (!latest.exists()) {
			latest.mkdirs();
		}
		File repository = new File(REPOSITORY);
		if (!repository.exists()) {
			repository.mkdirs();
		}
	}

    //HASHING

    /**
     * Generate a SHA-256 hash based on given string.
     * 
     * @param toHash
     *      The string to generate the hash from.
     * @return a SHA-256 hash based on given string.
     */
    private static String generateHash(String toHash) {
        String hash = "";
		try {
			byte[] encodedHash = MessageDigest.getInstance("SHA-256").digest(toHash.getBytes());
			hash = bytesToHexString(encodedHash);
		} catch (NoSuchAlgorithmException e) {
			e.printStackTrace();
		}
		return hash;
    }

	private static String bytesToHexString(byte[] byteArray) {
		StringBuffer hexString = new StringBuffer();
		for (int i = 0; i < byteArray.length; i++) {
			String hex = Integer.toHexString(0xff & byteArray[i]);
			if (hex.length() == 1)
				hexString.append('0');
			hexString.append(hex);
		}
		return hexString.toString();
	}
}
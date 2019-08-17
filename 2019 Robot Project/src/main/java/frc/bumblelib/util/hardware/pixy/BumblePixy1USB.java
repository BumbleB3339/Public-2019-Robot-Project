package frc.bumblelib.util.hardware.pixy;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * Class for opening a Pixy 1 sensor and retrieving data from it thorugh USB.
 */
public class BumblePixy1USB extends BumblePixy {

	public Pixy pixy;
	private int pixy_uid;

	/**
	 * Open a Pixy sensor and start reading data from it.
	 */
	public BumblePixy1USB(int uid) {
		this.pixy_uid = uid;

		if (!isConnected()) {
			DriverStation.reportError("Pixy is not connected!", true);
		}

		pixy = new Pixy(uid);
		pixy.startBlockProgram();
	}

	public void restartPixy() {
		pixy.stopBlockProgram();
		pixy.stopExecutor();
		pixy = new Pixy(pixy_uid);
		pixy.startBlockProgram();
	}

	/**
	 * Indicates whether the Pixy is currently connected.
	 * 
	 * @return Returns true if a valid connection is detected
	 */
	public boolean isConnected() {
		return Pixy.ensureAvailable(pixy_uid); 
	}

	/**
	 * A method for retrieving data from the Pixy.
	 */
	public PixyObjectList getObjects() {
		updatePixyData();
		return bufferedObjects.clone();
	}

	/**
	 * Retrieves data from the Pixy.
	 */
	protected void updatePixyData() {
		ArrayList<PixyBlock> blocks = new ArrayList<PixyBlock>(pixy.getBlocks());
		PixyObjectList newObjects = new PixyObjectList(blocks, REFLECTIVE_SIGNATURE, OBJECT_TIME_TO_LIVE);
		Collections.sort(newObjects); // sort by x
		bufferedObjects = newObjects;
	}
}

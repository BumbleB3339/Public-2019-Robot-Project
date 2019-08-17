package frc.bumblelib.util.hardware.pixy;

/**
 * Class for opening a Pixy or Pixy 2 sensor and retrieving data from it. This
 * code is based on https://github.com/croadfeldt/wpilib_pixy_spi_java.
 */
public abstract class BumblePixy {

	protected PixyObjectList bufferedObjects = new PixyObjectList();

	protected final int REFLECTIVE_SIGNATURE = 1;

	protected final int OBJECT_TIME_TO_LIVE = 1; // How many iterations an object is kept without receiving new data about it

	// Pixy 1 physical parameters
	public final static double PIXY1_HFOV = 75, PIXY1_VFOV = 47;
	public final static int PIXY1_FRAME_WIDTH = 320, PIXY1_FRAME_HEIGHT = 200;
	// ==========

	// Pixy 2 physical parameters
	public final static double PIXY2_HFOV = 60, PIXY2_VFOV = 40;
	public final static int PIXY2_FRAME_WIDTH = 316, PIXY2_FRAME_HEIGHT = 208;
	// ==========

	/**
	 * Indicates whether the Pixy is currently connected.
	 * 
	 * @return Returns true if a valid connection is detected
	 */
	public abstract boolean isConnected();

	/**
	 * A method for retrieving data from the Pixy.
	 */
	public PixyObjectList getObjects() {
		updatePixyData();
		return bufferedObjects.clone();
	}

	/**
	 * Retrieved data from the pixy and stores it in bufferedObects.
	 */
	protected abstract void updatePixyData();
}

package frc.bumblelib.util.hardware.pixy;

import java.util.Collections;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for opening a Pixy 2 sensor and retrieving data from it thorugh an
 * Arduino co-proccessor.
 */
public class BumblePixy2Arduino extends BumblePixy {

	private SerialPort serialPort;
	private boolean isArduinoConnected;

	/**
	 * Open a Pixy sensor and start reading data from it.
	 */
	public BumblePixy2Arduino() {
		SmartDashboard.putBoolean("Is Arduino connected", false);
		try {
			serialPort = new SerialPort(115200, Port.kUSB);
			SmartDashboard.putBoolean("Is Arduino connected", true);
			isArduinoConnected = true;
		} catch (Exception e) {
			DriverStation.reportError("Arduino is not connected!", true);
			isArduinoConnected = false;
		}
	}

	/**
	 * Indicates whether the Pixy is currently connected.
	 * 
	 * @return Returns true if a valid connection is detected
	 */
	public boolean isConnected() {
		return isArduinoConnected;
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
		if (!isArduinoConnected) {
			return;
		}

		// get block from arduino
		String str;
		try {
			serialPort.writeString("0");
			str = serialPort.readString();
		} catch (Exception e) {
			str = "";
		}

		// construct a PixyObjectList
		PixyObjectList newObjects;
		if (str != "") {
			newObjects = new PixyObjectList(str, REFLECTIVE_SIGNATURE, OBJECT_TIME_TO_LIVE);
		} else {
			newObjects = new PixyObjectList();
		}

		Collections.sort(newObjects); // sort by x
		bufferedObjects = newObjects;
	}
}

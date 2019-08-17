package frc.bumblelib.util.hardware;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;

public class Sharp_GP2Y0A41SK0F {
	private AnalogInput m_analogInput;
	private WPI_TalonSRX m_talonSRX;
	private boolean isTalon;

	/**
	 * Construct a Sharp-GP2Y0A41SK0F IR distance sensor
	 * 
	 * @param channel The channel number to represent. 0-3 are on-board 4-7 are on
	 *                the MXP port.
	 */
	public Sharp_GP2Y0A41SK0F(int channel) {
		m_analogInput = new AnalogInput(channel);
		isTalon = false;
	}

	/**
	 * Construct a Sharp-GP2Y0A41SK0F IR distance sensor
	 * 
	 * @param talonSRX The instance of the Talon SRX motor controller that the
	 *                 sensor is connected to.
	 */
	public Sharp_GP2Y0A41SK0F(WPI_TalonSRX talonSRX) {
		m_talonSRX = talonSRX;
		m_talonSRX.configSelectedFeedbackSensor(FeedbackDevice.Analog);
		isTalon = true;
	}

	/**
	 * Get distance reading from the sensor.
	 * 
	 * @return A double representing the distance reading in cm (4-40 cm range).
	 */
	public double getDistance() {
		return 12.691 * Math.pow(isTalon ? m_talonSRX.getSelectedSensorPosition() : m_analogInput.getAverageVoltage(),
				-1.089);
	}

	public double getVoltage() {
		return isTalon ? m_talonSRX.getSelectedSensorPosition() : m_analogInput.getVoltage();
	}
}

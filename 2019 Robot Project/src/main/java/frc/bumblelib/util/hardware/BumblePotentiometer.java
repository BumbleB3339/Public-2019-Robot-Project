package frc.bumblelib.util.hardware;

import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class BumblePotentiometer extends AnalogPotentiometer {
	private double firstValue = 0;
	private double firstVoltage = 0;
	private double secondValue = 0;
	private double secondVoltage = 0;

	private double minValidValue, maxValidValue;

	/**
	 * BumblePotentiometer constructor.
	 * 
	 * @param channel       The analog channel this potentiometer is plugged into.
	 * @param firstValue    First calibration VALUE.
	 * @param firstVoltage  First calibration VOLTAGE.
	 * @param secondValue   Second calibration VALUE.
	 * @param secondVoltage Second calibration VOLTAGE.
	 * @param minValidValue Minimum potentiometer VALUE to not be considered defective.
	 * @param maxValidValue Maximum potentiometer VALUE to not be considered defective.
	 */
	public BumblePotentiometer(int channel, double firstValue, double firstVoltage, double secondValue,
			double secondVoltage, double minValidValue, double maxValidValue) {
		super(channel);
		this.firstValue = firstValue;
		this.firstVoltage = firstVoltage;
		this.secondValue = secondValue;
		this.secondVoltage = secondVoltage;
		this.minValidValue = minValidValue;
		this.maxValidValue = maxValidValue;
	}

	/**
	 * BumblePotentiometer constructor.
	 * 
	 * @param channel       The analog channel this potentiometer is plugged into.
	 * @param firstValue    First calibration VALUE.
	 * @param firstVoltage  First calibration VOLTAGE.
	 * @param secondValue   Second calibration VALUE.
	 * @param secondVoltage Second calibration VOLTAGE.
	 */
	public BumblePotentiometer(int channel, double firstValue, double firstVoltage, double secondValue,
			double secondVoltage) {
		super(channel);
		this.firstValue = firstValue;
		this.firstVoltage = firstVoltage;
		this.secondValue = secondValue;
		this.secondVoltage = secondVoltage;
		this.minValidValue = -3339;
		this.maxValidValue = -3339;
	}

	/**
	 * BumblePotentiometer constructor.
	 * 
	 * @param channel The analog channel this potentiometer is plugged into.
	 */
	public BumblePotentiometer(int channel) {
		super(channel);
	}

	private double potConvert(double voltage) {
		double m = (firstValue - secondValue) / (firstVoltage - secondVoltage);
		return m * (voltage - firstVoltage) + firstValue;
	}

	/**
	 * Get the current position reading of the potentiometer.
	 *
	 * @return The current position of the potentiometer (in converted units).
	 */
	@Override
	public double get() {
		return potConvert(super.get());
	}

	public double getVoltage() {
		return super.get();
	}

	/**
	 * Get the defectiveness status of the potentiometer.
	 * 
	 * @return is the potentiometer defective.
	 */
	public boolean isDefective() {
		return get() > maxValidValue || get() < minValidValue;
	}

	@Override
	public double pidGet() {
		return get();
	}
}

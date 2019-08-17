package frc.bumblelib.util.hardware;

import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Construct a REV Analog Pressure Sensor
 */
public class REVAnalogPressureSensor extends AnalogInput {

    private final double VCC = 5.0; // Voltage supplied to the sensor.

    /**
     * Construct a REVAnalogPressureSensor.
     * 
     * @param channel The channel number to represent. 0-3 are on-board 4-7 are on
     *                the MXP port.
     */
    public REVAnalogPressureSensor(int channel) {
        super(channel);
    }

    /**
     * Get pressure reading from the sensor.
     * 
     * @return A double representing the pressure reading.
     */
    public double getPressure() {
        // Equation is derived from http://www.revrobotics.com/content/docs/REV-11-1107-DS.pdf
        
        return 250 * (getAverageVoltage() / VCC) - 25;
    }
}

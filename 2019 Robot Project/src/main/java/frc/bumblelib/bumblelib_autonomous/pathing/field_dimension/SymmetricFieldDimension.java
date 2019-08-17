package frc.bumblelib.bumblelib_autonomous.pathing.field_dimension;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;

/**
 * Contains four field measurements that mirror on the left and right side of
 * the field.
 */
public final class SymmetricFieldDimension extends FieldDimension {

    private double redRightSize;
    private double redLeftSize;

    private double blueRightSize;
    private double blueLeftSize;

    public SymmetricFieldDimension() {
    }

    /**
     * Creates a {@link SymmetricFieldDimension} containing four given measurements.
     * 
     * @param redRightSize  The measurements on the right side of the red alliance.
     * @param redLeftSize   The measurements on the left side of the red alliance.
     * @param blueRightSize The measurements on the right side of the blue alliance.
     * @param blueLeftSize  The measurements on the left side of the blue alliance.
     */
    private SymmetricFieldDimension(double redRightSize, double redLeftSize, double blueRightSize,
            double blueLeftSize) {
        this.redRightSize = redRightSize;
        this.redLeftSize = redLeftSize;

        this.blueRightSize = blueRightSize;
        this.blueLeftSize = blueLeftSize;
    }

    /**
     * Sets the value of right {@link Side} of the red {@link Alliance}'s
     * measurement.
     * 
     * @param redRightSize The value to set as the right {@link Side} of the red
     *                     {@link Alliance}'s measurement
     */
    public void setRedRightSize(double redRightSize) {
        this.redRightSize = redRightSize;
    }

    /**
     * Sets the value of left {@link Side} of the red {@link Alliance}'s
     * measurement.
     * 
     * @param redLeftSize The value to set as the left {@link Side} of the red
     *                    {@link Alliance}'s measurement
     */
    public void setRedLeftSize(double redLeftSize) {
        this.redLeftSize = redLeftSize;
    }

    /**
     * Sets the value of right {@link Side} of the blue {@link Alliance}'s
     * measurement.
     * 
     * @param blueRightSize The value to set as the right {@link Side} of the blue
     *                      {@link Alliance}'s measurement
     */
    public void setBlueRightSize(double blueRightSize) {
        this.blueRightSize = blueRightSize;
    }

    /**
     * Sets the value of left {@link Side} of the blue {@link Alliance}'s
     * measurement.
     * 
     * @param blueLeftSize The value to set as the left {@link Side} of the blue
     *                     {@link Alliance}'s measurement
     */
    public void setBlueLeftSize(double blueLeftSize) {
        this.blueLeftSize = blueLeftSize;
    }

    /**
     * Adds a given {@link SingularFieldDimension}'s measurment to both the red and
     * blue measurments of this {@link SymmetricFieldDimension}.
     * 
     * @param singularFieldDimension The {@link SingularFieldDimension} to add from.
     */
    public void addFieldDimension(SingularFieldDimension singularFieldDimension) {
        redRightSize += singularFieldDimension.getActualSize(Alliance.RED);
        redLeftSize += singularFieldDimension.getActualSize(Alliance.RED);

        blueRightSize += singularFieldDimension.getActualSize(Alliance.BLUE);
        blueLeftSize += singularFieldDimension.getActualSize(Alliance.BLUE);
    }

    /**
     * Adds the measurements of a given {@link SymmetricFieldDimension} to all the
     * measurements in the current object.
     * 
     * @param singularFieldDimension The {@link SingularFieldDimension} to add from.
     */
    public void addFieldDimension(SymmetricFieldDimension symmetricFieldDimension) {
        redRightSize += symmetricFieldDimension.getActualSize(Alliance.RED, Side.RIGHT);
        redLeftSize += symmetricFieldDimension.getActualSize(Alliance.RED, Side.LEFT);

        blueRightSize += symmetricFieldDimension.getActualSize(Alliance.BLUE, Side.RIGHT);
        blueLeftSize += symmetricFieldDimension.getActualSize(Alliance.BLUE, Side.LEFT);
    }

    /**
     * Returns a {@link SymmetricFieldDimension} with the opposite values of this
     * object.
     * 
     * @return A new {@link SymmetricFieldDimension} with the opposite values of
     *         this object's.
     */
    public SymmetricFieldDimension negative() {
        return new SymmetricFieldDimension(-redRightSize, -redLeftSize, -blueRightSize, -blueLeftSize);
    }

    /**
     * Returns a new {@link SymmetricFieldDimension} multiplied by a given value.
     * 
     * @param toMul The parameter to multply the {@link SymmetricFieldDimension}'s
     *              values by
     * @return A new {@link SymmetricFieldDimension} containing this object's values
     *         multiplied by a given parameter.
     */
    public SymmetricFieldDimension multiplyBy(double toMul) {
        return new SymmetricFieldDimension(redRightSize * toMul, redLeftSize * toMul, blueRightSize * toMul,
                blueLeftSize * toMul);
    }

    /**
     * Returns a value of this {@link SymmetricFieldDimension} at a given
     * {@link Side} and {@link Alliance}.
     * 
     * @return The value at the requested {@link Side} and {@link Alliance}.
     * @param alliance The {@link Alliance} of the requested value.
     * @param side     The {@link Side} of the requested value.
     */
    @Override
    public double getActualSize(Alliance alliance, Side side) {
        switch (alliance) {
        case RED:
            switch (side) {
            case RIGHT:
                return redRightSize;
            case LEFT:
                return redLeftSize;
            }
        case BLUE:
            switch (side) {
            case RIGHT:
                return blueRightSize;
            case LEFT:
                return blueLeftSize;
            }
        default:
            return 0.0;
        }
    }
}

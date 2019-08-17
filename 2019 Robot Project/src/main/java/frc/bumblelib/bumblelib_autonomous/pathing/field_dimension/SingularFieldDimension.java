package frc.bumblelib.bumblelib_autonomous.pathing.field_dimension;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;

/**
 * Contains a field dimension for both red and blue alliances.
 */
public final class SingularFieldDimension extends FieldDimension {

    private double redSize;
    private double blueSize;

    public SingularFieldDimension() {
    }

    /**
     * Creates a new {@link SingularFieldDimension} object containing a given
     * measurement for both red and blue alliances.
     * 
     * @param redSize  The value to set as the red side's measurement.
     * @param blueSize The value to set as the blue side's measurement.
     */
    private SingularFieldDimension(double redSize, double blueSize) {
        this.redSize = redSize;
        this.blueSize = blueSize;
    }

    /**
     * Sets a given value as the new red alliance's measurment.
     * 
     * @param redSize The value to set as the red alliance's measurment
     */
    public void setRedSize(double redSize) {
        this.redSize = redSize;
    }

    /**
     * Sets a given value as the new blue alliance's measurment.
     * 
     * @param blueSize The value to set as the blue alliance's measurment
     */
    public void setBlueSize(double blueSize) {
        this.blueSize = blueSize;
    }

    /**
     * Adds a given {@link SingularFieldDimension}'s measurment to both the red and
     * blue measurments of this {@link SingularFieldDimension}.
     * 
     * @param singularFieldDimension The {@link SingularFieldDimension} to add from.
     */
    public void addFieldDimension(SingularFieldDimension singularFieldDimension) {
        redSize += singularFieldDimension.getActualSize(Alliance.RED);
        blueSize += singularFieldDimension.getActualSize(Alliance.BLUE);
    }

    /**
     * Adds the a given {@link SymmetricFieldDimension} to the given side of this
     * {@link SingularFieldDimension}.
     * 
     * @param symmetricFieldDimension The {@link SymmetricFieldDimension} to add
     *                                from.
     * @param side                    The {@link Side} of the given
     *                                {@link SymmetricFieldDimension} to add from.
     */
    public void addFieldDimension(SymmetricFieldDimension symmetricFieldDimension, Side side) {
        redSize += symmetricFieldDimension.getActualSize(Alliance.RED, side);
        blueSize += symmetricFieldDimension.getActualSize(Alliance.BLUE, side);
    }

    /**
     * Returns a new {@link SingularFieldDimension} contianing this
     * {@link SingularFieldDimension}'s negative values.
     * 
     * @return A new {@link SingularFieldDimension} containing this
     *         {@link SingularFieldDimension}'s negative values.
     */
    public SingularFieldDimension negative() {
        return new SingularFieldDimension(-redSize, -blueSize);
    }

    /**
     * Returns a new {@link SingularFieldDimension} with this
     * {@link SingularFieldDimension}'s values multiplied by given value.
     * 
     * @param toMul The value to multiply by.
     * @return A new {@link SingularFieldDimension} with this
     *         {@link SingularFieldDimension}'s values multiplied by given value.
     */
    public SingularFieldDimension multiplyBy(double toMul) {
        return new SingularFieldDimension(redSize * toMul, blueSize * toMul);
    }

    /**
     * Returns the value of a given {@link Alliance}'s measurement.
     * 
     * @param alliance The {@link Alliance} of the wanted measurement.
     * @return The the value of the given {@link Alliance}'s measurement.
     */
    public double getActualSize(Alliance alliance) {
        switch (alliance) {
        case RED:
            return redSize;
        case BLUE:
            return blueSize;
        default:
            return 0.0;
        }
    }

    /**
     * Returns the value of a given {@link Alliance}'s measurement.
     * 
     * @param alliance The {@link Alliance} of the wanted measurement.
     * @param side     The {@link Side} of the wanted measurement (doesn't matter
     *                 because it's singular).
     * @return The the value of the given {@link Alliance}'s measurement.
     */
    @Override
    public double getActualSize(Alliance alliance, Side side) {
        return this.getActualSize(alliance);
    }
}

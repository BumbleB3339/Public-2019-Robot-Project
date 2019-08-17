package frc.bumblelib.bumblelib_autonomous.pathing.field_dimension;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;

/**
 * A Field Dimension contains a field measurment for both red and blue alliances
 * and optionaly left and right sides of the field
 */
public abstract class FieldDimension {
    /**
     * Get the actual size of the measurment for a specific {@link Alliance} and
     * {@link Side}
     * 
     * @param alliance The {@link Alliance} of the wanted value.
     * @param side     The {@link Side} of the wanted value.
     * @return The actual size of the measurment.
     */
    public abstract double getActualSize(Alliance alliance, Side side);
}

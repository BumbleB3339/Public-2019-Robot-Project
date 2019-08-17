package frc.bumblelib.bumblelib_autonomous.pathing.field_point;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.bumblelib.bumblelib_autonomous.pathing.field_dimension.SingularFieldDimension;
import frc.bumblelib.bumblelib_autonomous.pathing.field_dimension.SymmetricFieldDimension;

public final class SingularFieldPoint extends FieldPoint {

    private double redX;
    private double redY;

    private double blueX;
    private double blueY;

    public SingularFieldPoint() {
    }

    /**
     * Creates a new {@link SingularFieldPoint} with the given values.
     * 
     * @param redX  The value to set as the red {@link Side}'s X.
     * @param blueX The value to set as the blue {@link Side}'s X.
     * @param redY  The value to set as the red {@link Side}'s Y.
     * @param blueY The value to set as the blue {@link Side}'s Y.
     */
    public SingularFieldPoint(double redX, double blueX, double redY, double blueY) {
        this.redX = redX;
        this.blueX = blueX;

        this.redY = redY;
        this.blueY = blueY;
    }

    /**
     * Creates a new {@link SingularFieldPoint} using an existing
     * {@link SymmetricFieldPoint}.
     * 
     * @param symmetricFieldPoint The {@link SymmetricFieldPoint} to copy from.
     * @param side                The {@link Side} of the
     *                            {@link SymmetricFieldPoint} to copy from.
     */
    public SingularFieldPoint(SymmetricFieldPoint symmetricFieldPoint, Side side) {
        this.redX = symmetricFieldPoint.getX(Alliance.RED, side);
        this.blueX = symmetricFieldPoint.getX(Alliance.BLUE, side);

        this.redY = symmetricFieldPoint.getY(Alliance.RED, side);
        this.blueY = symmetricFieldPoint.getY(Alliance.BLUE, side);
    }

    /**
     * Adds a given value to the Y value of a given {@link Side} and
     * {@link Alliance} of this {@link SingularFieldPoint}.
     * 
     * @param ySize    The value to add.
     * @param alliance The {@link Alliance} of the Y value to add to.
     */
    private void addY(double ySize, Alliance alliance) {
        switch (alliance) {
        case RED:
            this.redY += ySize;
            break;
        case BLUE:
            this.blueY += ySize;
            break;
        }
    }

    /**
     * Adds a given value the X value of this {@link SingularFieldPoint}'s given
     * {@link Alliance}.
     * 
     * @param xSize    The value to add.
     * @param alliance The {@link Alliance} of the X value to add to.
     */
    private void addX(double xSize, Alliance alliance) {
        switch (alliance) {
        case RED:
            this.redX += xSize;
            break;
        case BLUE:
            this.blueX += xSize;
            break;
        }
    }

    /**
     * Returns a new {@link SingularFieldPoint} containing the Y value of this
     * {@link SingularFieldPoint}'s Y value with an added given value.
     * 
     * @param dY The value to add.
     * @return A new {@link SingularFieldPoint} containing the Y value of this
     *         {@link SingularFieldPoint}'s Y value with an added given value.
     */
    public SingularFieldPoint addY(double dY) {
        addY(dY, Alliance.RED);
        addY(dY, Alliance.BLUE);

        return this;
    }

    /**
     * Returns a new {@link SingularFieldPoint} containing this
     * {@link SingularFieldPoint}'s Y value subtracted by a given value.
     * 
     * @param dY the value to subtract.
     * @return A new {@link SingularFieldPoint} containing this
     *         {@link SingularFieldPoint}'s Y value minus a given value.
     */
    public SingularFieldPoint subtractY(double dY) {
        addY(-dY);

        return this;
    }

    /**
     * Returns a new {@link SingularFieldPoint} containing this
     * {@link SingularFieldPoint} Y values with the added Y value of a given
     * {@link SingularFieldDimension}'s value.
     * 
     * @param singularFieldDimension the {@link SingularFieldDimension} to add from.
     * @return A new {@link SingularFieldDimension} containing this
     *         {@link SingularFieldPoint} Y values plus the Y value of a given
     *         {@link SingularFieldDimension}'s value.
     */
    public SingularFieldPoint addY(SingularFieldDimension singularFieldDimension) {
        addY(singularFieldDimension.getActualSize(Alliance.RED), Alliance.RED);
        addY(singularFieldDimension.getActualSize(Alliance.BLUE), Alliance.BLUE);

        return this;
    }

    /**
     * Returns a new {@link SingularFieldPoint} containing this
     * {@link SingularFieldPoint}'s Y value subtracted by a given
     * {@link SingularFieldDimension}'s value.
     * 
     * @param singularFieldDimension The {@link SingularFieldDimension} to subtract
     *                               with.
     * @return A new {@link SingularFieldPoint} containing this
     *         {@link SingularFieldPoint}'s Y value subtracted by a given
     *         {@link SingularFieldDimension}'s value.
     */
    public SingularFieldPoint subtractY(SingularFieldDimension singularFieldDimension) {
        addY(singularFieldDimension.negative());

        return this;
    }

    /**
     * Returns a new {@link SingularFieldPoint} containing this
     * {@link SingularFieldPoint}'s Y value with an added Y value from a given
     * {@link SymmetricFieldDimension}'s value on a given side of the
     * {@link SymmetricFieldDimension}.
     * 
     * @param symmetricFieldDimension The {@link SymmetricFieldDimension} to add
     *                                from.
     * @param side                    The {@link Side} of the
     *                                {@link SymmetricFieldDimension} to add from.
     * @return A new {@link SingularFieldPoint} containing this
     *         {@link SingularFieldPoint}'s Y value with an added Y value from a
     *         given {@link SymmetricFieldDimension}'s value on a given side of the
     *         {@link SymmetricFieldDimension}.
     */
    public SingularFieldPoint addY(SymmetricFieldDimension symmetricFieldDimension, Side side) {
        addY(symmetricFieldDimension.getActualSize(Alliance.RED, side), Alliance.RED);
        addY(symmetricFieldDimension.getActualSize(Alliance.BLUE, side), Alliance.BLUE);

        return this;
    }

    /**
     * Returns a new {@link SingularFieldPoint} which contains this
     * {@link SingularFieldPoint}'s Y value minus a given
     * {@link SymmetricFieldDimension}'s value at a given {@link Side}.
     * 
     * @param symmetricFieldDimension The {@link SymmetricFieldDimension} to get the
     *                                subtracted value from.
     * @param side                    The side of the
     *                                {@link SymmetricFieldDimension}'s wanted
     *                                value.
     * @return A new {@link SingularFieldPoint} which contains this
     *         {@link SingularFieldPoint}'s Y value minus a given
     *         {@link SymmetricFieldDimension}'s value at a given {@link Side}.
     */
    public SingularFieldPoint subtractY(SymmetricFieldDimension symmetricFieldDimension, Side side) {
        addY(symmetricFieldDimension.negative(), side);

        return this;
    }

    /**
     * Returns a new {@link SingularFieldPoint} containing this
     * {@link SingularFieldPoint}'s same Y value and an X value with a given
     * addition to the X value of this {@link SingularFieldPoint}.
     * 
     * @param dX The value to add to the X value of the {@link SingularFieldPoint}.
     * @return A new {@link SingularFieldPoint} containing this
     *         {@link SingularFieldPoint}'s same Y value and an X value with a given
     *         addition to the X value of this {@link SingularFieldPoint}.
     */
    public SingularFieldPoint addX(double dX) {
        addX(dX, Alliance.RED);
        addX(dX, Alliance.BLUE);

        return this;
    }

    /**
     * Returns a new {@link SingularFieldPoint} containing this
     * {@link SingularFieldPoint}'s same Y value and an X value with a given
     * subtraction to the X value of this {@link SingularFieldPoint}.
     * 
     * @param dX The value to subtract from the X value.
     * @return A new {@link SingularFieldPoint} containing this
     *         {@link SingularFieldPoint}'s same Y value and an X value with a given
     *         subtraction to the X value of this {@link SingularFieldPoint}.
     */
    public SingularFieldPoint subtractX(double dX) {
        addX(-dX);

        return this;
    }

    /**
     * Returns a new {@link SingularFieldPoint} containing this
     * {@link SingularFieldPoint}'s same Y value and an X that has an addition from
     * a {@link SingularFieldDimension}'s measurement.
     * 
     * @param singularFieldDimension The {@link SingularFieldDimension} to add from.
     * @return A new {@link SingularFieldPoint} containing this
     *         {@link SingularFieldPoint}'s same Y value and an X that has an
     *         addition from a {@link SingularFieldDimension}'s measurement.
     */
    public SingularFieldPoint addX(SingularFieldDimension singularFieldDimension) {
        addX(singularFieldDimension.getActualSize(Alliance.RED), Alliance.RED);
        addX(singularFieldDimension.getActualSize(Alliance.BLUE), Alliance.BLUE);

        return this;
    }

    /**
     * Returns a new {@link SingularFieldPoint} containing this
     * {@link SingularFieldPoint}'s same Y value and an X made of this
     * {@link SingularFieldPoint}'s value subtracted by the measurement of a given
     * {@link SingularFieldDimension}.
     * 
     * @param singularFieldDimension The {@link SingularFieldDimension} to subtract
     *                               from.
     * @return A new {@link SingularFieldPoint} containing this
     *         {@link SingularFieldPoint}'s same Y value and an X made of this
     *         {@link SingularFieldPoint}'s value subtracted by the measurement of a
     *         given {@link SingularFieldDimension}.
     */
    public SingularFieldPoint subtractX(SingularFieldDimension singularFieldDimension) {
        addX(singularFieldDimension.negative());

        return this;
    }

    /**
     * Returns a new {@link SingularFieldPoint} containing this
     * {@link SingularFieldPoint}'s same Y value and an X that has an addition from
     * a {@link SymmetricFieldDimension}'s measurement at a given {@link Side}.
     * 
     * @param symmetricFieldDimension The {@link SymmetricFieldDimension} to add
     *                                from.
     * @param side                    The {@link Side} of the given
     *                                {@link SymmetricFieldDimension}'s measurement
     *                                to add from.
     * @return A new {@link SingularFieldPoint} containing this
     *         {@link SingularFieldPoint}'s same Y value and an X that has an
     *         addition from a {@link SymmetricFieldDimension}'s measurement at a
     *         given {@link Side}.
     */
    public SingularFieldPoint addX(SymmetricFieldDimension symmetricFieldDimension, Side side) {
        addX(symmetricFieldDimension.getActualSize(Alliance.RED, side), Alliance.RED);
        addX(symmetricFieldDimension.getActualSize(Alliance.BLUE, side), Alliance.BLUE);

        return this;
    }

    /**
     * Returns a new {@link SingularFieldPoint} containing this
     * {@link SingularFieldPoint}'s same Y value and an X made of this
     * {@link SingularFieldPoint}'s value subtracted by the measurement of a given
     * {@link SymmetricFieldDimension}.
     * 
     * @param symmetricFieldDimension The {@link SymmetricFieldDimension} to
     *                                subtract from.
     * @param side                    The {@link Side} of the given
     *                                {@link SymmetricFieldDimension}'s measurement
     *                                to subtract.
     * @return A new {@link SingularFieldPoint} containing this
     *         {@link SingularFieldPoint}'s same Y value and an X made of this
     *         {@link SingularFieldPoint}'s value subtracted by the measurement of a
     *         given {@link SymmetricFieldDimension}.
     */
    public SingularFieldPoint subtractX(SymmetricFieldDimension symmetricFieldDimension, Side side) {
        addX(symmetricFieldDimension.negative(), side);

        return this;
    }

    /**
     * Returns a new {@link SingularFieldPoint} containing this
     * {@link SingularFieldPoint}'s values (X and Y) with an added factor to it.
     * 
     * @param dX The X factor to add.
     * @param dY The Y factor to add.
     * @return A new {@link SingularFieldPoint} containing this
     *         {@link SingularFieldPoint}'s values (X and Y) with an added factor to
     *         it.
     */
    public SingularFieldPoint displace(double dX, double dY) {
        return new SingularFieldPoint(this.redX + dX, this.blueX + dX, this.redY + dY, this.blueY + dY);
    }

    /**
     * Returns a new {@link SingularFieldPoint} shifted by a given distance in a
     * given angle relatively to this {@link SingularFieldPoint}'s (X,Y) coordinates
     * (as if it's a vector with the size set as the given distance and direction
     * set as the given angle).
     * 
     * @param distance The distance to shift the {@link SingularFieldPoint}.
     * @param angle    The angle to shift the {@link SingularFieldPoint}.
     * @return A new {@link SingularFieldPoint} shifted by a given distance in a
     *         given angle relatively to this {@link SingularFieldPoint}'s (X,Y)
     *         coordinates (as if it's a vector with the size set as the given
     *         distance and direction set as the given angle).
     */
    public SingularFieldPoint displaceByAngle(double distance, double angle) {
        double dX = -1 * distance * Math.sin(Math.toRadians(angle));
        double dY = distance * Math.cos(Math.toRadians(angle));

        return new SingularFieldPoint(this.redX + dX, this.blueX + dX, this.redY + dY, this.blueY + dY);
    }

    /**
     * Returns the X values of the {@link SingularFieldPoint} at a given
     * {@link Alliance}.
     * 
     * @param alliance The {@link Alliance} of the requested X value.
     * @return The X values of the {@link SingularFieldPoint} at a given
     *         {@link Alliance}.
     */
    public double getX(Alliance alliance) {
        switch (alliance) {
        case RED:
            return redX;
        case BLUE:
            return blueX;
        default:
            return 0.0;
        }
    }

    /**
     * Returns the Y values of the {@link SingularFieldPoint} at a given
     * {@link Alliance}.
     * 
     * @param alliance The {@link Alliance} of the requested Y value.
     * @return The Y values of the {@link SingularFieldPoint} at a given
     *         {@link Alliance}.
     */
    public double getY(Alliance alliance) {
        switch (alliance) {
        case RED:
            return redY;
        case BLUE:
            return blueY;
        default:
            return 0.0;
        }
    }

    /**
     * Returns the X values of the {@link SingularFieldPoint} at a given
     * {@link Alliance}.
     * 
     * @param alliance The {@link Alliance} of the requested X value.
     * @return The X values of the {@link SingularFieldPoint} at a given
     *         {@link Alliance}.
     */
    @Override
    public double getX(Alliance alliance, Side side) {
        return this.getX(alliance);
    }

    /**
     * Returns the Y values of the {@link SingularFieldPoint} at a given
     * {@link Alliance}.
     * 
     * @param alliance The {@link Alliance} of the requested Y value.
     * @return The Y values of the {@link SingularFieldPoint} at a given
     *         {@link Alliance}.
     */
    @Override
    public double getY(Alliance alliance, Side side) {
        return this.getY(alliance);
    }
}

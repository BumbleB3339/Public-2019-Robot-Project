package frc.bumblelib.bumblelib_autonomous.pathing.field_point;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;
import frc.bumblelib.bumblelib_autonomous.pathing.field_dimension.SingularFieldDimension;
import frc.bumblelib.bumblelib_autonomous.pathing.field_dimension.SymmetricFieldDimension;
import frc.robot.profiles.InitProfiles;

public final class SymmetricFieldPoint extends FieldPoint {

    private double redRightX;
    private double redRightY;

    private double redLeftX;
    private double redLeftY;

    private double blueRightX;
    private double blueRightY;

    private double blueLeftX;
    private double blueLeftY;

    /**
     * Creates a new {@link SymmetricFieldPoint} with a default X value of half the
     * field's width.
     */
    public SymmetricFieldPoint() {
        this.redRightX = InitProfiles.FIELD_PROFILE.getFieldWidth();
        this.blueRightX = InitProfiles.FIELD_PROFILE.getFieldWidth();
    }

    /**
     * Creates a new {@link SymmetricFieldPoint} using the givem parameters.
     * 
     * @param redRightX  The X value to set at the red {@link Alliance} on the right
     *                   {@link Side}.
     * @param blueRightX The X value to set at the blue {@link Alliance} on the
     *                   right {@link Side}.
     * @param redRightY  The Y value to set at the red {@link Alliance} on the right
     *                   {@link Side}.
     * @param blueRightY The Y value to set at the blue {@link Alliance} on the
     *                   right {@link Side}.
     * @param redLeftX   The X value to set at the red {@link Alliance} on the left
     *                   {@link Side}.
     * @param blueLeftX  The X value to set at the blue {@link Alliance} on the left
     *                   {@link Side}.
     * @param redLeftY   The Y value to set at the red {@link Alliance} on the left
     *                   {@link Side}.
     * @param blueLeftY  The Y value to set at the blue {@link Alliance} on the left
     *                   {@link Side}.
     */
    public SymmetricFieldPoint(double redRightX, double blueRightX, double redRightY, double blueRightY,
            double redLeftX, double blueLeftX, double redLeftY, double blueLeftY) {
        this.redRightX = redRightX;
        this.blueRightX = blueRightX;

        this.redRightY = redRightY;
        this.blueRightY = blueRightY;

        this.redLeftX = redLeftX;
        this.blueLeftX = blueLeftX;

        this.redLeftY = redLeftY;
        this.blueLeftY = blueLeftY;
    }

    /**
     * Adds a given Y of the {@link SymmetricFieldPoint} at given {@link Side} of
     * the given {@link Alliance}.
     * 
     * @param ySize    The Y value to add.
     * @param alliance The {@link Alliance} of the Y.
     * @param side     The {@link Side} of the Y.
     */
    private void addY(double ySize, Alliance alliance, Side side) {
        switch (alliance) {
        case RED:
            switch (side) {
            case RIGHT:
                this.redRightY += ySize;
                break;
            case LEFT:
                this.redLeftY += ySize;
                break;
            }
            break;
        case BLUE:
            switch (side) {
            case RIGHT:
                this.blueRightY += ySize;
                break;
            case LEFT:
                this.blueLeftY += ySize;
                break;
            }
            break;
        }
    }

    /**
     * Adds a given X of the {@link SymmetricFieldPoint} at given {@link Side} of
     * the given {@link Alliance}.
     * 
     * @param xSize    The X value to add.
     * @param alliance The {@link Alliance} of the X.
     * @param side     The {@link Side} of the X.
     */
    private void addX(double xSize, Alliance alliance, Side side) {
        switch (alliance) {
        case RED:
            switch (side) {
            case RIGHT:
                this.redRightX -= xSize;
                break;
            case LEFT:
                this.redLeftX += xSize;
                break;
            }
            break;
        case BLUE:
            switch (side) {
            case RIGHT:
                this.blueRightX -= xSize;
                break;
            case LEFT:
                this.blueLeftX += xSize;
                break;
            }
            break;
        }
    }

    /**
     * Returns a new {@link SymmetricFieldPoint} contains the Y value of this
     * {@link SymmetricFieldPoint} plus the given Y value.
     * 
     * @param dY The Y value to add.
     * @return A new {@link SymmetricFieldPoint} contains the Y value of this
     *         {@link SymmetricFieldPoint} plus the given Y value.
     */
    public SymmetricFieldPoint addY(double dY) {
        addY(dY, Alliance.RED, Side.RIGHT);
        addY(dY, Alliance.BLUE, Side.RIGHT);

        addY(dY, Alliance.RED, Side.LEFT);
        addY(dY, Alliance.BLUE, Side.LEFT);

        return this;
    }

    /**
     * Returns a new {@link SymmetricFieldPoint} contains the Y value of this
     * {@link SymmetricFieldPoint} minus the given Y value.
     * 
     * @param dY The Y value to subtract.
     * @return A new {@link SymmetricFieldPoint} contains the Y value of this
     *         {@link SymmetricFieldPoint} minus the given Y value.
     */
    public SymmetricFieldPoint subtractY(double dY) {
        addY(-dY);

        return this;
    }

    /**
     * Returns a new {@link SymmetricFieldPoint} containing this
     * {@link SymmetricFieldPoint}'s Y values with an addition of a given
     * {@link SingularFieldDimension}'s value.
     * 
     * @param singularFieldDimension The {@link SingularFieldDimension} to add from.
     * @return A new {@link SymmetricFieldPoint} containing this
     *         {@link SymmetricFieldPoint}'s Y values with an added Y value of a
     *         given {@link SingularFieldDimension}'s value.
     */
    public SymmetricFieldPoint addY(SingularFieldDimension singularFieldDimension) {
        addY(singularFieldDimension.getActualSize(Alliance.RED), Alliance.RED, Side.RIGHT);
        addY(singularFieldDimension.getActualSize(Alliance.BLUE), Alliance.BLUE, Side.RIGHT);

        addY(singularFieldDimension.getActualSize(Alliance.RED), Alliance.RED, Side.LEFT);
        addY(singularFieldDimension.getActualSize(Alliance.BLUE), Alliance.BLUE, Side.LEFT);

        return this;
    }

    /**
     * Returns a new {@link SymmetricFieldPoint} containing this
     * {@link SymmetricFieldPoint}'s Y value subtracted by a given
     * {@link SingularFieldDimension}'s value.
     * 
     * @param singularFieldDimension The {@link SingularFieldDimension} to subtract
     *                               with.
     * @return A new {@link SymmetricFieldPoint} containing this
     *         {@link SymmetricFieldPoint}'s Y value subtracted by a given
     *         {@link SingularFieldDimension}'s value.
     */
    public SymmetricFieldPoint subtractY(SingularFieldDimension singularFieldDimension) {
        addY(singularFieldDimension.negative());

        return this;
    }

    /**
     * Returns a new {@link SymmetricFieldPoint} containing this
     * {@link SymmetricFieldPoint}'s Y value with an added Y value from a given
     * {@link SymmetricFieldDimension}'s value on a given side of the
     * {@link SymmetricFieldDimension}.
     * 
     * @param symmetricFieldDimension The {@link SymmetricFieldDimension} to add
     *                                from.
     * @return A new {@link SingularFieldPoint} containing this
     *         {@link SingularFieldPoint}'s Y value with an added Y value from a
     *         given {@link SymmetricFieldDimension}'s value on a given side of the
     *         {@link SymmetricFieldDimension}.
     */
    public SymmetricFieldPoint addY(SymmetricFieldDimension symmetricFieldDimension) {
        addY(symmetricFieldDimension.getActualSize(Alliance.RED, Side.RIGHT), Alliance.RED, Side.RIGHT);
        addY(symmetricFieldDimension.getActualSize(Alliance.BLUE, Side.RIGHT), Alliance.BLUE, Side.RIGHT);

        addY(symmetricFieldDimension.getActualSize(Alliance.RED, Side.LEFT), Alliance.RED, Side.LEFT);
        addY(symmetricFieldDimension.getActualSize(Alliance.BLUE, Side.LEFT), Alliance.BLUE, Side.LEFT);

        return this;
    }

    /**
     * Returns a new {@link SymmetricFieldPoint} which contains this
     * {@link SymmetricFieldPoint}'s Y value minus a given
     * {@link SymmetricFieldDimension}'s value at a given {@link Side}.
     * 
     * @param symmetricFieldDimension The {@link SymmetricFieldDimension} to get the
     *                                subtracted value from.
     * @return A new {@link SymmetricFieldPoint} which contains this
     *         {@link SymmetricFieldPoint}'s Y value minus a given
     *         {@link SymmetricFieldDimension}'s value at a given {@link Side}.
     */
    public SymmetricFieldPoint subtractY(SymmetricFieldDimension symmetricFieldDimension) {
        addY(symmetricFieldDimension.negative());

        return this;
    }

    /**
     * Returns a new {@link SymmetricFieldPoint} containing this
     * {@link SymmetricFieldPoint}'s X values with an addition of a given value.
     * 
     * @param dX The value to add.
     * @return A new {@link SymmetricFieldPoint} containing this
     *         {@link SymmetricFieldPoint}'s X values with an addition of a given
     *         value.
     */
    public SymmetricFieldPoint addX(double dX) {
        addX(dX, Alliance.RED, Side.RIGHT);
        addX(dX, Alliance.BLUE, Side.RIGHT);

        addX(dX, Alliance.RED, Side.LEFT);
        addX(dX, Alliance.BLUE, Side.LEFT);

        return this;
    }

    /**
     * Returns a new {@link SymmetricFieldPoint} containing this
     * {@link SymmetricFieldPoint}'s same Y value and the X value of this
     * {@link SymmetricFieldPoint} minus a given value.
     * 
     * @param dX The value to subtract from the X value.
     * @return A new {@link SymmetricFieldPoint} containing this
     *         {@link SymmetricFieldPoint}'s same Y value and the X value of this
     *         {@link SymmetricFieldPoint} minus a given value.
     */
    public SymmetricFieldPoint subtractX(double dX) {
        addX(-dX);

        return this;
    }

    /**
     * Returns a new {@link SymmetricFieldPoint} containing this
     * {@link SymmetricFieldPoint}'s same Y value and an X that has an addition from
     * a {@link SingularFieldDimension}'s measurement.
     * 
     * @param singularFieldDimension The {@link SingularFieldDimension} to add from.
     * @return A new {@link SymmetricFieldPoint} containing this
     *         {@link SymmetricFieldPoint}'s same Y value and an X that has an
     *         addition from a {@link SingularFieldDimension}'s measurement.
     */
    public SymmetricFieldPoint addX(SingularFieldDimension singularFieldDimension) {
        addX(singularFieldDimension.getActualSize(Alliance.RED), Alliance.RED, Side.RIGHT);
        addX(singularFieldDimension.getActualSize(Alliance.BLUE), Alliance.BLUE, Side.RIGHT);

        addX(singularFieldDimension.getActualSize(Alliance.RED), Alliance.RED, Side.LEFT);
        addX(singularFieldDimension.getActualSize(Alliance.BLUE), Alliance.BLUE, Side.LEFT);

        return this;
    }

    /**
     * Returns a new {@link SymmetricFieldPoint} containing this
     * {@link SymmetricFieldPoint}'s same Y value and an X made of this
     * {@link SymmetricFieldPoint}'s value subtracted by the measurement of a given
     * {@link SingularFieldDimension}.
     * 
     * @param singularFieldDimension The {@link SingularFieldDimension} to subtract
     *                               from.
     * @return A new {@link SymmetricFieldPoint} containing this
     *         {@link SymmetricFieldPoint}'s same Y value and an X made of this
     *         {@link SymmetricFieldPoint}'s value subtracted by the measurement of
     *         a given {@link SingularFieldDimension}.
     */
    public SymmetricFieldPoint subtractX(SingularFieldDimension singularFieldDimension) {
        addX(singularFieldDimension.negative());

        return this;
    }

    /**
     * Returns a new {@link SymmetricFieldPoint} containing this
     * {@link SymmetricFieldPoint}'s same Y value and an X that has an addition from
     * a {@link SymmetricFieldDimension}'s measurements.
     * 
     * @param symmetricFieldDimension The {@link SymmetricFieldDimension} to add
     *                                from.
     * @return A new {@link SymmetricFieldPoint} containing this
     *         {@link SymmetricFieldPoint}'s same Y value and an X that has an
     *         addition from a {@link SymmetricFieldDimension}'s measurements.
     */
    public SymmetricFieldPoint addX(SymmetricFieldDimension symmetricFieldDimension) {
        addX(symmetricFieldDimension.getActualSize(Alliance.RED, Side.RIGHT), Alliance.RED, Side.RIGHT);
        addX(symmetricFieldDimension.getActualSize(Alliance.BLUE, Side.RIGHT), Alliance.BLUE, Side.RIGHT);

        addX(symmetricFieldDimension.getActualSize(Alliance.RED, Side.LEFT), Alliance.RED, Side.LEFT);
        addX(symmetricFieldDimension.getActualSize(Alliance.BLUE, Side.LEFT), Alliance.BLUE, Side.LEFT);

        return this;
    }

    /**
     * Returns a new {@link SingularFieldPoint} which is based on this
     * {@link SymmetricFieldPoint}'s values at a given {@link Side}.
     * 
     * @param side The {@link Side} to copy values from.
     * @return A new {@link SingularFieldPoint} which is based on this
     *         {@link SymmetricFieldPoint}'s values at a given {@link Side}.
     */
    public SingularFieldPoint getSingular(Side side) {
        return new SingularFieldPoint(this, side);
    }

    /**
     * Returns a new {@link SymmetricFieldPoint} containing this
     * {@link SymmetricFieldPoint}'s same Y value and an X made of this
     * {@link SymmetricFieldPoint}'s value subtracted by the measurement of a given
     * {@link SymmetricFieldDimension}.
     * 
     * @param symmetricFieldDimension The {@link SymmetricFieldDimension} to
     *                                subtract from.
     * @return Returns a new {@link SymmetricFieldPoint} containing this
     *         {@link SymmetricFieldPoint}'s same Y value and an X made of this
     *         {@link SymmetricFieldPoint}'s value subtracted by the measurement of
     *         a given {@link SymmetricFieldDimension}.
     */
    public SymmetricFieldPoint subtractX(SymmetricFieldDimension symmetricFieldDimension) {
        addX(symmetricFieldDimension.negative());

        return this;
    }

    /**
     * Returns a new {@link SymmetricFieldPoint} containing this
     * {@link SymmetricFieldPoint}'s values (X and Y) with an added factor to it.
     * 
     * @param dX The X factor to add.
     * @param dY The Y factor to add.
     * @return A new {@link SymmetricFieldPoint} containing this
     *         {@link SymmetricFieldPoint}'s values (X and Y) with an added factor
     *         to it.
     */
    public SymmetricFieldPoint displace(double dX, double dY) {
        return new SymmetricFieldPoint(redRightX + dX, blueRightX + dX, redRightY + dY, blueRightY + dY, redLeftX - dX,
                blueLeftX - dX, redLeftY + dY, blueLeftY + dY);
    }

    /**
     * Returns a new {@link SymmetricFieldPoint} shifted by a given distance in a
     * given angle relatively to this {@link SymmetricFieldPoint}'s (X,Y)
     * coordinates (as if it's a vector with the size set as the given distance and
     * direction set as the given angle).
     * 
     * @param distance The distance to shift the {@link SymmetricFieldPoint}.
     * @param angle    The angle to shift the {@link SymmetricFieldPoint}. A
     *                 positive angle represents an outside rotation.
     * @return A new {@link SymmetricFieldPoint} shifted by a given distance in a
     *         given angle relatively to this {@link SymmetricFieldPoint}'s (X,Y)
     *         coordinates (as if it's a vector with the size set as the given
     *         distance and direction set as the given angle).
     */
    public SymmetricFieldPoint displaceByAngle(double distance, double angle) {
        double dX = distance * Math.sin(Math.toRadians(angle));
        double dY = distance * Math.cos(Math.toRadians(angle));

        return new SymmetricFieldPoint(redRightX + dX, blueRightX + dX, redRightY + dY, blueRightY + dY, redLeftX - dX,
                blueLeftX - dX, redLeftY + dY, blueLeftY + dY);
    }

    /**
     * Returns the X values of the {@link SymmetricFieldPoint} at a given
     * {@link Side} of a given {@link Alliance}.
     * 
     * @param alliance The {@link Alliance} of the requested X value.
     * @param side     The {@link Side} of the reqeusted X value.
     * @return The X values of the {@link SymmetricFieldPoint} at a given
     *         {@link Side} of a given {@link Alliance}.
     */
    @Override
    public double getX(Alliance allilance, Side side) {
        switch (allilance) {
        case RED:
            switch (side) {
            case RIGHT:
                return redRightX;
            case LEFT:
                return redLeftX;
            }
        case BLUE:
            switch (side) {
            case RIGHT:
                return blueRightX;
            case LEFT:
                return blueLeftX;
            }
        default:
            return 0.0;
        }
    }

    /**
     * Returns the Y values of the {@link SymmetricFieldPoint} at a given
     * {@link Side} of a given {@link Alliance}.
     * 
     * @param alliance The {@link Alliance} of the requested Y value.
     * @param side     The {@link Side} of the reqeusted Y value.
     * @return The Y values of the {@link SymmetricFieldPoint} at a given
     *         {@link Side} of a given {@link Alliance}.
     */
    @Override
    public double getY(Alliance allilance, Side side) {
        switch (allilance) {
        case RED:
            switch (side) {
            case RIGHT:
                return redRightY;
            case LEFT:
                return redLeftY;
            }
        case BLUE:
            switch (side) {
            case RIGHT:
                return blueRightY;
            case LEFT:
                return blueLeftY;
            }
        default:
            return 0.0;
        }
    }
}

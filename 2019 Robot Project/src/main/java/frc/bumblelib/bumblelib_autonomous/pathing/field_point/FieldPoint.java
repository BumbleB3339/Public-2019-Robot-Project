package frc.bumblelib.bumblelib_autonomous.pathing.field_point;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;

public abstract class FieldPoint {

    public abstract double getX(Alliance allilance, Side side);

    public abstract double getY(Alliance allilance, Side side);
}

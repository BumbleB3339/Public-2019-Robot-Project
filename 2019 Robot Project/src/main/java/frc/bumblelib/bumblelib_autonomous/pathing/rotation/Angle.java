package frc.bumblelib.bumblelib_autonomous.pathing.rotation;

import frc.bumblelib.bumblelib_autonomous.pathing.enums.Alliance;
import frc.bumblelib.bumblelib_autonomous.pathing.enums.Side;

public abstract class Angle {
    
    public abstract int getValue(Alliance alliance, Side side);
}
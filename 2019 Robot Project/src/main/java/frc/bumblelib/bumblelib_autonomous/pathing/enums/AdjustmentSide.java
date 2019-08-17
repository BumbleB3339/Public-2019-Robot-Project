package frc.bumblelib.bumblelib_autonomous.pathing.enums;

public enum AdjustmentSide {
    RIGHT, LEFT, BOTH;

    public AdjustmentSide getOtherSide() {
        switch (this) {
        case RIGHT:
            return LEFT;
        case LEFT:
            return RIGHT;
        default:
            return BOTH;
        }
    }
}

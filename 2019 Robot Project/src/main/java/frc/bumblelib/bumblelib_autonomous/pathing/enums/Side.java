package frc.bumblelib.bumblelib_autonomous.pathing.enums;

public enum Side {
    RIGHT, LEFT;

    public Side getOtherSide() {
        switch (this) {
        case RIGHT:
            return LEFT;
        case LEFT:
            return RIGHT;
        default:
            return null;
        }
    }
}

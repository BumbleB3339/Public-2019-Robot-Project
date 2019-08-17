package frc.bumblelib.util.hardware;

import edu.wpi.first.wpilibj.DigitalInput;

public class BumbleSwitch extends DigitalInput{

    private boolean isNormallyClosed;
    public BumbleSwitch(int channel, boolean isNormallyClosed){
        super(channel);
        this.isNormallyClosed = isNormallyClosed;
    }

    @Override
    public boolean get() {
        return isNormallyClosed ? !super.get() : super.get();
    }
}

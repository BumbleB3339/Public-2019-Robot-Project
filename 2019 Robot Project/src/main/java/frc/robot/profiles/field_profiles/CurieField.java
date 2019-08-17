package frc.robot.profiles.field_profiles;

import frc.bumblelib.bumblelib_autonomous.pathing.field_dimension.SingularFieldDimension;
import frc.bumblelib.bumblelib_autonomous.pathing.field_dimension.SymmetricFieldDimension;
import frc.robot.profiles.FieldProfile;

public class CurieField extends FieldProfile {

    @Override
    public double getFieldWidth() {
        return 8.18;
    }

    @Override
    public SymmetricFieldDimension sideWallToLevel1FlatEdge() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.setBlueLeftSize(2.43);
        sfd.setBlueRightSize(2.46);
        sfd.setRedLeftSize(2.49);
        sfd.setRedRightSize(2.44);
        return sfd;
    }

    @Override
    public SymmetricFieldDimension sideWallToRocket() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.setBlueLeftSize(0.20);
        sfd.setBlueRightSize(0.19);
        sfd.setRedLeftSize(0.19);
        sfd.setRedRightSize(0.20);
        return sfd;
    }

    @Override
    public SymmetricFieldDimension allianceWallToCloseRocketCorner() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.setBlueLeftSize(5.36);
        sfd.setBlueRightSize(5.37);
        sfd.setRedLeftSize(5.37);
        sfd.setRedRightSize(5.38);
        return sfd;
    }

    @Override
    public SymmetricFieldDimension sideWallToMidFeeder() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.setBlueLeftSize(0.66);
        sfd.setBlueRightSize(0.66);
        sfd.setRedLeftSize(0.66);
        sfd.setRedRightSize(0.66);
        return sfd;
    }

    @Override
    public SingularFieldDimension level1FlatEdgeToFrontCargoShipFace() {
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(3.45);
        sfd.setRedSize(3.47);
        return sfd;
    }

    @Override
    public SymmetricFieldDimension sideWallToCargoshipSide() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.setBlueLeftSize(3.38);
        sfd.setBlueRightSize(3.39);
        sfd.setRedLeftSize(3.39);
        sfd.setRedRightSize(3.37);
        return sfd;
    }
}

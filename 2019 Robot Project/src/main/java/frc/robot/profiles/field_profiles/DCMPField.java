package frc.robot.profiles.field_profiles;

import frc.bumblelib.bumblelib_autonomous.pathing.field_dimension.SingularFieldDimension;
import frc.bumblelib.bumblelib_autonomous.pathing.field_dimension.SymmetricFieldDimension;
import frc.robot.profiles.FieldProfile;

public class DCMPField extends FieldProfile {

    @Override
    public double getFieldWidth() {
        return 8.18;
    }

    @Override
    public SymmetricFieldDimension sideWallToLevel1FlatEdge() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.setBlueLeftSize(2.44);
        sfd.setBlueRightSize(2.45);
        sfd.setRedLeftSize(2.48);
        sfd.setRedRightSize(2.46);
        return sfd;  
    }

    @Override
    public SymmetricFieldDimension sideWallToRocket() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.setBlueLeftSize(0.19);
        sfd.setBlueRightSize(0.18);
        sfd.setRedLeftSize(0.120);
        sfd.setRedRightSize(0.19);
        return sfd;  
    }

    @Override
    public SymmetricFieldDimension allianceWallToCloseRocketCorner() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.setBlueLeftSize(5.31);
        sfd.setBlueRightSize(5.29);
        sfd.setRedLeftSize(5.24);
        sfd.setRedRightSize(5.31);
        return sfd;  
    }

    @Override
    public SymmetricFieldDimension sideWallToMidFeeder() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.setBlueLeftSize(0.65);
        sfd.setBlueRightSize(0.65);
        sfd.setRedLeftSize(0.65);
        sfd.setRedRightSize(0.65);
        return sfd;  
    }

    @Override
    public SingularFieldDimension level1FlatEdgeToFrontCargoShipFace() {
        SingularFieldDimension sfd = new SingularFieldDimension();
        sfd.setBlueSize(3.25);
        sfd.setRedSize(3.25);
        return sfd; 
    }

    @Override
    public SymmetricFieldDimension sideWallToCargoshipSide() {
        SymmetricFieldDimension sfd = new SymmetricFieldDimension();
        sfd.setBlueLeftSize(3.43);
        sfd.setBlueRightSize(3.41);
        sfd.setRedLeftSize(3.43);
        sfd.setRedRightSize(3.41);
        return sfd;
    }
}
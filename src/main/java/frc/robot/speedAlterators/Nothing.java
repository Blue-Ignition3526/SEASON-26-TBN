package frc.robot.speedAlterators;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lib.BlueShift.control.SpeedAlterator;

public class Nothing extends SpeedAlterator {
    public Nothing() { }

    @Override
    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) { return speeds; }
}

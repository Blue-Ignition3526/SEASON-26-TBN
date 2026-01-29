package frc.robot.speedAlterators;

import java.util.function.Supplier;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static frc.robot.Constants.SwerveDriveConstants.PoseControllers.rotationPID;
import lib.BlueShift.control.SpeedAlterator;

public class LookToward extends SpeedAlterator {
    private final Supplier<Pose2d> odoSupplier;
    private final Point target;

    public LookToward(Supplier<Pose2d> odoSupplier, Point target) {
        this.target = target;
        this.odoSupplier = odoSupplier;
        rotationPID.enableContinuousInput(0, Math.PI*2);
    }

    @Override
    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        double angle = Math.atan2(odoSupplier.get().getY() - target.y, odoSupplier.get().getX() - target.x);
        double omegaRadiansPerSecond = rotationPID.calculate(odoSupplier.get().getRotation().getRadians(), angle);

        return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omegaRadiansPerSecond);
    }
}

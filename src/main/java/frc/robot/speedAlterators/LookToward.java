package frc.robot.speedAlterators;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static frc.robot.Constants.SwerveDriveConstants.PoseControllers.rotationPID;
import lib.BlueShift.control.SpeedAlterator;

public class LookToward extends SpeedAlterator {
    private final Supplier<Pose2d> odoSupplier;
    private final Translation2d target;

    public LookToward(Supplier<Pose2d> odoSupplier, Translation2d target) {
        this.target = target;
        this.odoSupplier = odoSupplier;
    }
    
    @Override
    public void onEnable() {
        rotationPID.reset(odoSupplier.get().getRotation().getRadians());
        Logger.recordOutput("LookTowards/Target", target);
    }

    @Override
    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        double angle =  (Math.atan2(odoSupplier.get().getY() - target.getY(), odoSupplier.get().getX() - target.getX()) + Math.PI);
        Logger.recordOutput("LookTowards/Angle", angle);
        double omegaRadiansPerSecond = rotationPID.calculate(odoSupplier.get().getRotation().getRadians(), angle);

        return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omegaRadiansPerSecond);
    }
}

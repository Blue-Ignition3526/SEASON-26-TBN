package frc.robot.speedAlterators;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.Constants;

public class ByHeading extends SpeedAlterator {
    private final Supplier<Double> angleSupplier;

    private final Supplier<Double> targetAngle;

    public ByHeading(Supplier<Double> angleSupplier, Supplier<Double> targetAngle) {
        this.angleSupplier = angleSupplier;
        this.targetAngle = targetAngle;
        SmartDashboard.putBoolean("Alterators/Look", false);
        Constants.SwerveDriveConstants.PoseControllers.rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void onEnable() {
        SmartDashboard.putBoolean("Alterators/Look", true);
    }

    @Override
    public void onDisable() {
        SmartDashboard.putBoolean("Alterators/Look", false);
    }

    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        double rotSpeed = Constants.SwerveDriveConstants.PoseControllers.rotationPID.calculate(angleSupplier.get(), targetAngle.get());
        SmartDashboard.putNumber("Alterators/DesiredAngle", targetAngle.get());
        SmartDashboard.putNumber("Alterators/CurrentAngle", angleSupplier.get());
        SmartDashboard.putNumber("Alterators/Speed", rotSpeed);

        speeds.omegaRadiansPerSecond = rotSpeed;

        return speeds;
    }
}
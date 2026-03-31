package frc.robot.speedAlterators;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDriveConstants.PoseControllers;

public class Shake extends SpeedAlterator {
    private final Rotation2d magnitude;
    private Rotation2d originalHeading;
    private final Supplier<Rotation2d> angleSupplier;
    private boolean left = false;

    private final ControlType controlType = ControlType.BANG_BANG;

    enum ControlType {
        PID,
        BANG_BANG
    }

    public Shake(Rotation2d magnitude, Supplier<Rotation2d> angleSupplier) {
        this.magnitude = magnitude;
        this.originalHeading = angleSupplier.get();
        this.angleSupplier = angleSupplier;
    }

    @Override
    public void onEnable() {
        PoseControllers.rotationPID.reset(angleSupplier.get().getRadians());
        this.originalHeading = angleSupplier.get();
    }

    @Override
    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        Rotation2d setpoint = left ? Rotation2d.fromRadians(originalHeading.getRadians() + magnitude.getRadians()) : Rotation2d.fromRadians(originalHeading.getRadians() - magnitude.getRadians());
        
        if(left) {
            if(setpoint.getRadians() < angleSupplier.get().getRadians() /*% 2*Math.PI*/) {
                left = false;
            }
        } else {
            if(setpoint.getRadians() > angleSupplier.get().getRadians() /*% 2*Math.PI*/) {
                left = true;
            }
        }

        double omegaRadiansPerSecond = 0;
        if(controlType == ControlType.PID) {
                omegaRadiansPerSecond = PoseControllers.rotationPID.calculate(angleSupplier.get().getRadians(), setpoint.getRadians());
        } else {
            if(left) {
                    omegaRadiansPerSecond = Constants.SwerveDriveConstants.PhysicalModel.kMaxAngularSpeed.minus(RadiansPerSecond.of(3)).in(RadiansPerSecond);
            } else {
                    omegaRadiansPerSecond = -Constants.SwerveDriveConstants.PhysicalModel.kMaxAngularSpeed.minus(RadiansPerSecond.of(3)).in(RadiansPerSecond);
            }
        }

        
        return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omegaRadiansPerSecond);
    }
}

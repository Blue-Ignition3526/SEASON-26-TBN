package frc.robot.speedAlterators;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.Constants;

public class KeepHeading extends SpeedAlterator {
    private final Supplier<Pose2d> poseSupplier;
    private final double epsilonRadians;
    private double targetAngle;

    public KeepHeading(Supplier<Pose2d> poseSupplier, double epsilonRadians) {
        this.poseSupplier = poseSupplier;
        this.epsilonRadians = epsilonRadians;

        Constants.SwerveDriveConstants.PoseControllers.rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    @Override
    public void onEnable() {
        targetAngle = poseSupplier.get().getRotation().getRadians();
        Constants.SwerveDriveConstants.PoseControllers.rotationPID.reset(poseSupplier.get().getRotation().getRadians());
    }

    @Override
    public void onDisable() { Constants.SwerveDriveConstants.PoseControllers.rotationPID.reset(poseSupplier.get().getRotation().getRadians()); }

    boolean lastIterationMoving = false;
    boolean updatedTarget = false;
    double timeSinceStop;

    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        if(Math.abs(speeds.omegaRadiansPerSecond) <= epsilonRadians) {
            if(lastIterationMoving) {
                timeSinceStop = System.currentTimeMillis();

                lastIterationMoving = false;
                updatedTarget = false;
            }

            //? Try using Gyro to set when real angular speed is < epsilon
            if(timeSinceStop + 500 <= System.currentTimeMillis() && !updatedTarget) {
                targetAngle = poseSupplier.get().getRotation().getRadians();
                Constants.SwerveDriveConstants.PoseControllers.rotationPID.reset(poseSupplier.get().getRotation().getRadians());

                updatedTarget = true;
            }

            double speed = Constants.SwerveDriveConstants.PoseControllers.rotationPID.calculate((poseSupplier.get().getRotation().getRadians()), targetAngle);
            if(Math.abs(speed) >= epsilonRadians && updatedTarget) {
                speeds.omegaRadiansPerSecond = speed;
            }
        } else {
            lastIterationMoving = true;
            updatedTarget = false;
        }

        return speeds;
    }
}
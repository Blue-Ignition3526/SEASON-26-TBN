package frc.robot.speedAlterators;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.BlueShift.control.SpeedAlterator;
import frc.robot.Constants;

public class Rock extends SpeedAlterator {
    private final Supplier<Pose2d> poseSuplier;
    private final Pose2d targetPose;
    private Pose2d originalPose;

    public Rock(Supplier<Pose2d> poseSuplier) {
        this.poseSuplier = poseSuplier;
        this.originalPose = poseSuplier.get();
        this.targetPose = poseSuplier.get().times(1.1);
    }

    @Override
    public void onEnable() {
        Pose2d pose = poseSuplier.get();
        Constants.SwerveDriveConstants.PoseControllers.translationXPID.reset(pose.getX());
        Constants.SwerveDriveConstants.PoseControllers.translationYPID.reset(pose.getY());
        Constants.SwerveDriveConstants.PoseControllers.rotationPID.reset(pose.getRotation().getRotations());

        this.originalPose = poseSuplier.get().times(1.1);
    }

    @Override
    public void onDisable() {}

    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        Pose2d pose = poseSuplier.get();

        double xSpeed = Constants.SwerveDriveConstants.PoseControllers.translationXPID.calculate(pose.getX(), targetPose.getX());
        double ySpeed = Constants.SwerveDriveConstants.PoseControllers.translationYPID.calculate(pose.getY(), targetPose.getY());
        double rotSpeed = Constants.SwerveDriveConstants.PoseControllers.rotationPID.calculate(pose.getRotation().getRotations(), targetPose.getRotation().getRotations());
       
        SmartDashboard.putNumber("Alterators/pose/rot", rotSpeed);
        SmartDashboard.putNumber("Alterators/pose/x", xSpeed);
        SmartDashboard.putNumber("Alterators/pose/y", ySpeed);
        Logger.recordOutput("Alterators/pose/desiredPose", targetPose);

        return ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, pose.getRotation());
    }
}

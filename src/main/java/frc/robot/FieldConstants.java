package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
    public static final AprilTagFieldLayout kApriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final double kFieldWidth = kApriltagFieldLayout.getFieldWidth();
    public static final double kFieldLength = kApriltagFieldLayout.getFieldLength();
    public static final Translation2d kFieldCenter = new Translation2d(kFieldLength / 2, kFieldWidth / 2);

    public static void logCalculatedPoses() {}
}
package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
    public static final AprilTagFieldLayout kApriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final double kFieldWidth = kApriltagFieldLayout.getFieldWidth();
    public static final double kFieldLength = kApriltagFieldLayout.getFieldLength();
    public static final Translation2d kFieldCenter = new Translation2d(kFieldLength / 2, kFieldWidth / 2);
    
    public static final Translation2d kBlueHubPosition = new Translation2d(4.64, kFieldWidth/2.0);
    public static final Translation2d kRedHubPosition = new Translation2d(11.91, kFieldWidth/2.0);

    public static void logCalculatedPoses() {}
}

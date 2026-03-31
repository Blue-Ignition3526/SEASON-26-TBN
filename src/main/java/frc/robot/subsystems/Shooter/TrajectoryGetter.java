package frc.robot.subsystems.Shooter;

import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import java.util.Map;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.FieldConstants;
import lib.BlueShift.math.BlueMathUtils;

public class TrajectoryGetter {
    public static double getSetpointWithMap(Pose2d pose) {
        Distance distance = Meters.of(pose.getTranslation().getDistance(hubPos()));

        double[] mapValues = findInMap(distance);
        Distance[] mapKeys = getKeysInMap(distance);

        if (mapValues.length == 1) {
            return mapValues[0];
        }

        double t = distance.minus(mapKeys[0]).div(mapKeys[1]).magnitude();
        double setpoint = BlueMathUtils.lerp(mapValues[0], mapValues[1], t);

        Logger.recordOutput("Shooter/Debug/distance", distance);
        Logger.recordOutput("Shooter/Debug/t", t);

        return setpoint;
    }

    // This should go in utils.
    private static double[] findInMap(Distance target) {
        Map.Entry<Distance, Double> lower = kSpeedsMap.floorEntry(target);
        Map.Entry<Distance, Double> higher = kSpeedsMap.ceilingEntry(target);

        if (lower == null && higher == null) {
            return null;
        }
        if (lower == null) {
            return new double[] { higher.getValue() };
        }
        if (higher == null) {
            return new double[] { lower.getValue() };
        }

        // If target exactly matches a key
        if (lower.getKey().equals(higher.getKey())) {
            return new double[] { lower.getValue() };
        }

        return new double[] { lower.getValue(), higher.getValue() };
    }

    static Distance[] getKeysInMap(Distance target) {
        Distance higher = kSpeedsMap.ceilingKey(target);
        Distance lower = kSpeedsMap.floorKey(target);

        if (lower == null && higher == null) {
            return null;
        }
        if (lower == null) {
            return new Distance[] { higher };
        }
        if (higher == null) {
            return new Distance[] { lower };
        }

        return new Distance[] { lower, higher };
    }

    public static Translation2d hubPos() {
        return DriverStation.getAlliance().get() == Alliance.Red ? FieldConstants.kRedHubPosition
                : FieldConstants.kBlueHubPosition;
    }
}

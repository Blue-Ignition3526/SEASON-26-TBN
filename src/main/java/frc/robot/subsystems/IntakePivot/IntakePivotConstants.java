package frc.robot.subsystems.IntakePivot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;

public class IntakePivotConstants {
    public static final int kMotorId = -1;

    public static final int kCurrentLimit = 40;

    public static final double kConversionFactor = 1/25;

    // ! Values not set
    public static final Angle kMinAngle = Degrees.of(0);
    public static final Angle kMaxAngle = Degrees.of(0);

    public static final Angle kManualPos0 = Degrees.of(0);
    public static final Angle kManualPos1 = Degrees.of(0);
    public static final Angle kManualPos2 = Degrees.of(0);
    public static final Angle kManualPos3 = Degrees.of(0);

    public static final PIDController kPID = new PIDController(0, 0, 0);
    public static final Angle kEpsilon = Degrees.of(1);
    public static final double kLimit = 0;
}

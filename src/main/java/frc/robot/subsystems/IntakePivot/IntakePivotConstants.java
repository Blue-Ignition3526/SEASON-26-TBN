package frc.robot.subsystems.IntakePivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;

public class IntakePivotConstants {
    public static final int kMotorId = 51;
    public static final int kEncoderId = 52;

    public static final int kCurrentLimit = 40;

    // ! Values not set
    // Limits
    public static final Angle kMinAngle = Rotations.of(0);
    public static final Angle kMaxAngle = Rotations.of(0.4);

    // Positions
    public static final Angle kPosDown = Rotations.of(0.01);
    public static final Angle kPosUp = Rotations.of(0.42);

    public static final Angle kManualPos0 = Degrees.of(0);
    public static final Angle kManualPos1 = Degrees.of(0);
    public static final Angle kManualPos2 = Degrees.of(0);
    public static final Angle kManualPos3 = Degrees.of(0);

    // PID
    public static final PIDController kPID = new PIDController(25, 0, 0);
    public static final Angle kEpsilon = Degrees.of(25);
    public static final double kLimit = 11;
}

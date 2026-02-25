package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;

public class IntakeConstants {
    public static final int kIntakeMotorId = -1;

    // * Copied from HopperConstants
    public static final int kCurrentLimit = 30;
    // * Absolute guess
    public static final int kIntakeVoltage = 5;

    // ! Might be wrong and should be 1/25
    public static final double kConversionFactor = 25;

    // ! Values not set
    public static final Angle kPivotMinAngle = Degrees.of(0);
    public static final Angle kPivotMaxAngle = Degrees.of(0);

    public static final PIDController kPivotPID = new PIDController(-1, -1, -1);
}

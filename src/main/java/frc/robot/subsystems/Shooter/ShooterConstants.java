package frc.robot.subsystems.Shooter;

import java.util.TreeMap;

import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public class ShooterConstants {
  public static final int kLeftMotorId = 30;
  public static final int kRightMotorID = 31;

  public static final int kCurrentLimit = 40;
  public static final double kRampRate = 0.2;

  public static final double manual1 = 30;
  public static final double manual2 = 48;
  public static final double manual3 = 62;
  public static final double manual4 = 75;

  public static final double kIdleSpeed = manual1;

  public static final class MagicShooterConstants {
    public static final double kS = 0.0;
    public static final double kV = 0.12;
    public static final double kA = 0.0;
    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kEpsilon = 2;
  }


  // TODO: calibrate automatic speeds (RPS)
  public static final TreeMap<Distance, Double> kSpeedsMap = new TreeMap<>();
  static {
    kSpeedsMap.put(Meters.of(0.0), 41.5);
    kSpeedsMap.put(Meters.of(1.5), 41.5);
    kSpeedsMap.put(Meters.of(2.0), 44.5);
    kSpeedsMap.put(Meters.of(2.5), 46.5);
    kSpeedsMap.put(Meters.of(3.0), 47.5);
    kSpeedsMap.put(Meters.of(3.5), 51.5);
    kSpeedsMap.put(Meters.of(4.0), 54.5);
    kSpeedsMap.put(Meters.of(5.0), 57.5);
    kSpeedsMap.put(Meters.of(6.0), 68.5);
    kSpeedsMap.put(Meters.of(999999.9999), 68.5);
  }


  // This shooter switches control type, from bang bang to currentTorque
  // Using bang bang ensures fastest acceleration to setpoint
  // Using Current ensures consistent ball exit velocity
  public static final class CoolShooterConstants {
    public enum ShooterState {
      SPINUP, // Bang-Bang
      READY, // Bang-Bang
      FUEL_CONTACT, // Torque
      RECOVERY // Bang-Bang
    }

    //! This decides when to switch from Torque control back to bang bang
    public static final double kEpsilon = 5;

    //! This value needs to be found experimentaly
    //! Can be the value of ball passing in normal mode + sefety margin
    //? Change this to idle - peak to get the ball current, and then scale idle depending on setpoint
    public static final double kShootCurrent = 37;
  }
}

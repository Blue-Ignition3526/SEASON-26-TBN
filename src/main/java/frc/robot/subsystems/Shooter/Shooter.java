package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import lib.BlueShift.math.BlueMathUtils;

import static frc.robot.subsystems.Shooter.ShooterConstants.*;
import static edu.wpi.first.units.Units.*;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  /**
  * Master
  */
  private final TalonFX leftMotor;

  /**
  * Slave  <br/>
  * WARNING Never send control requests to this motor, only on setup
  */
  private final TalonFX s_rightMotor;

  private double setpoint;
  private final VelocityVoltage controlRequest;
  private final Supplier<Pose2d> poseSuppler;

  private boolean automatic = true;

  public Shooter(Supplier<Pose2d> poseSupplier) {
    leftMotor = new TalonFX(kLeftMotorId);
    s_rightMotor = new TalonFX(kRightMotorID);

    TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration()
    .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
    ).withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(Amps.of(120))
        .withSupplyCurrentLimit(Amps.of(40))
        .withStatorCurrentLimitEnable(true)
        .withSupplyCurrentLimitEnable(true)
    ).withSlot0(new Slot0Configs()
      .withKS(kS)
      .withKV(kV)
      .withKA(kA)
      .withKP(kP)
      .withKI(kI)
      .withKD(kD)
    );

    TalonFXConfiguration rightMotorConfig = leftMotorConfig.clone();

    leftMotor.getConfigurator().apply(leftMotorConfig);
    s_rightMotor.getConfigurator().apply(rightMotorConfig);

    this.controlRequest = new VelocityVoltage(0);

    leftMotor.setControl(this.controlRequest);
    s_rightMotor.setControl(new Follower(kLeftMotorId, MotorAlignmentValue.Opposed));

    this.poseSuppler = poseSupplier;
  }

  public void set(double rps) {
    leftMotor.setControl(controlRequest.withVelocity(rps));
  }

  public void stop() {
    // This should also stop the follower
    leftMotor.stopMotor();
  }

  public double getRPS() {
    return leftMotor.getVelocity().getValueAsDouble();
  }

  public boolean ready() {
    return Math.abs(getRPS() - setpoint) < kEpsilon;
  }

  public double getSetpointWithMap(Pose2d pose) {
    Distance distance = Meters.of(pose.getTranslation().getDistance(FieldConstants.kHubPosition));

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

  double[] findInMap(Distance target) {
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

  Distance[] getKeysInMap(Distance target) {
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

    return new Distance[] { lower, higher};
  }
  
  public Command setCommand(double rps) {
    return run(()->{automatic = false; set(rps);});
  }

  public Command shootCommand() {
    return runEnd(()->{automatic=true;}, this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public void configureOrchestra(Orchestra orchestra) {
    orchestra.addInstrument(leftMotor);
    orchestra.addInstrument(s_rightMotor);
  }

  @Override
  public void periodic() {
    if(automatic) {
      setpoint = getSetpointWithMap(poseSuppler.get());
      // Check setpoint gives expected values, convert if needed.
      // set(setpoint);
    }
    
    Logger.recordOutput("Shooter/Speed", getRPS());
    Logger.recordOutput("Shooter/setpoint", setpoint);
  }
}

package frc.robot.subsystems.Hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.Hopper.HopperConstants.*;

public class Hopper extends SubsystemBase {
  private final SparkFlex motor;

  public Hopper() {
    this.motor = new SparkFlex(kMotorID, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();

    config
      .smartCurrentLimit(kCurrentLimit)
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void set(double voltage) {
    motor.setVoltage(voltage);
  }

  public void hopperate() {
    set(hopperationVoltage);
  }

  public void stop() {
    motor.stopMotor();
  }

  public Command hopperationCommand() {
    return runEnd(this::hopperate, this::stop);
  }
}

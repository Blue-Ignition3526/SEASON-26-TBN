package frc.robot.subsystems.Hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.Hopper.HopperConstants.*;

public class Hopper extends SubsystemBase {
  private final SparkFlex hopperMotor;
  // private final SparkMax upperMotor;

  public Hopper() {
    this.hopperMotor = new SparkFlex(kHopperMotorID, MotorType.kBrushless);
    // // this.upperMotor = new SparkMax(kUpperMotorID, MotorType.kBrushless);

    SparkFlexConfig hopperConfig = new SparkFlexConfig();
    SparkMaxConfig upperConfig = new SparkMaxConfig();

    hopperConfig
      .smartCurrentLimit(kCurrentLimit)
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    
    upperConfig
      .smartCurrentLimit(kCurrentLimit)
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    hopperMotor.configure(hopperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    // upperMotor.configure(upperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void hopperSet(double voltage) {
    hopperMotor.setVoltage(voltage);
  }

  public void upperSet(double voltage) {
    // upperMotor.setVoltage(voltage);
  }

  public void hopperate() {
    hopperSet(kHopperationVoltage);
    upperSet(kUpperationVoltage);
  }

  public void reverse() {
    hopperSet(-kHopperationVoltage);
    upperSet(-kUpperationVoltage);
  }

  public void stop() {
    hopperMotor.stopMotor();
    // upperMotor.stopMotor();
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public Command hopperationCommand() {
    return runEnd(this::hopperate, this::stop);
  }

  public Command hopperationCommandForAuto() {
    return run(this::hopperate);
  }

  public Command reverseCommand() {
    return runEnd(this::reverse, this::stop);
  }
}

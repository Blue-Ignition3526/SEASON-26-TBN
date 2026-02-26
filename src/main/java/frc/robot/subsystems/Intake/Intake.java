// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.Intake.IntakeConstants.*;

public class Intake extends SubsystemBase {
  private final SparkFlex motor;

  public Intake() {
    this.motor = new SparkFlex(kMotorId, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();
    config
      .smartCurrentLimit(kCurrentLimit)
      .idleMode(IdleMode.kBrake)
      .inverted(false);

    this.motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Intake Functions
  public void set(double voltage) {
    this.motor.setVoltage(voltage);
  }

  public void setIn() {
    set(kInVoltage);
  }

  public void setOut() {
    set(-kOutVoltage);
  }

  public void stop() {
    this.motor.stopMotor();
  }

  // Commands
  public Command setInCommand() {
    return runOnce(this::setIn);
  }

  public Command setOutCommand() {
    return runOnce(this::setOut);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}

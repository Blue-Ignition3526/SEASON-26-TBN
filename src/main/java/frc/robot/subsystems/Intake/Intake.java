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

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.Intake.IntakeConstants.*;

import java.util.function.Supplier;

public class Intake extends SubsystemBase {
  private final SparkFlex intakeMotor;
  private final SparkFlex pivotMotor;

  private Angle pivotSetpoint = Degrees.of(0);

  public Intake() {
    this.intakeMotor = new SparkFlex(kIntakeMotorId, MotorType.kBrushless);
    this.pivotMotor = new SparkFlex(kCurrentLimit, MotorType.kBrushless);

    SparkFlexConfig intakeConfig = new SparkFlexConfig();
    intakeConfig
      .smartCurrentLimit(kCurrentLimit)
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    
    SparkFlexConfig pivotConfig = new SparkFlexConfig();
    pivotConfig
      .smartCurrentLimit(kCurrentLimit)
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    pivotConfig.absoluteEncoder
      .velocityConversionFactor(kConversionFactor)
      .positionConversionFactor(kConversionFactor);

    this.intakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    this.pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Intake Functions
  public void set(double voltage) {
    this.intakeMotor.setVoltage(voltage);
  }

  public void setIn() {
    set(kIntakeVoltage);
  }

  public void setOut() {
    set(-kIntakeVoltage);
  }

  public void stop() {
    set(0);
  }

  // Pivot Functions
  public Angle getAngle() {
    // ? Should Absolute Encoder be used?
    return Degrees.of(pivotMotor.getAbsoluteEncoder().getPosition()*360);
  }

  public void pivotSet(double voltage) {
    this.pivotMotor.setVoltage(voltage);
  }

  public void pivotSetSetpoint(Supplier<Angle> angle) {
    pivotSetpoint = angle.get();
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

  public Command pivotSetSetpointCommand(Supplier<Angle> setpoint) {
    return runOnce(() -> this.pivotSetSetpoint(setpoint));
  }

  @Override
  public void periodic() {
    if (pivotSetpoint.in(Degrees) < kPivotMinAngle.in(Degrees) || pivotSetpoint.in(Degrees) > kPivotMaxAngle.in(Degrees)) {
      // Out of bounds
      DriverStation.reportError(pivotSetpoint.in(Degrees) + ": Is outside of the pivot range", null);
      return;
    }

    pivotSet(kPivotPID.calculate(getAngle().in(Degrees), this.pivotSetpoint.in(Degrees)));
  }
}

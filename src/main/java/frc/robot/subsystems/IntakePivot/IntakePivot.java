// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePivot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.BlueShift.math.BlueMathUtils;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.IntakePivot.IntakePivotConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final SparkFlex motor;

  private Angle setpoint;
  private boolean enabled = true;

  private double PIDOutput;
  private boolean shouldRun;

  public IntakePivot() {
    this.motor = new SparkFlex(kMotorId, MotorType.kBrushless);

    SparkFlexConfig config = new SparkFlexConfig();
    config
      .smartCurrentLimit(kCurrentLimit)
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    config.absoluteEncoder
      .velocityConversionFactor(kConversionFactor)
      .positionConversionFactor(kConversionFactor);

    this.motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Angle getAngle() {
    // ? Should Absolute Encoder be used?
    return Rotations.of(motor.getAbsoluteEncoder().getPosition());
  }

  private void set(double voltage) {
    this.motor.setVoltage(voltage);
  }

  public void stop() {
    this.enabled = false;
    this.motor.stopMotor();
  }

  public void setSetpoint(Angle angle) {
    if (angle.in(Rotations) > kMinAngle.in(Rotations) || angle.in(Rotations) < kMaxAngle.in(Rotations)) {
      // In bounds
      this.enabled = true;
      this.setpoint = angle;
    }
  }

  // Commands
  public Command setSetpointCommand(Supplier<Angle> setpoint) {
    return run(() -> this.setSetpoint(setpoint.get()));
  }

  public Command setSetpointCommand(Angle setpoint) {
    return runOnce(() -> this.setSetpoint(setpoint));
  }

  public Command setUpCommand() {
    return runOnce(() -> this.setSetpoint(kPosUp));
  }

  public Command setDownCommand() {
    return runOnce(() -> this.setSetpoint(kPosDown));
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  @Override
  public void periodic() {
    shouldRun = this.enabled && Math.abs(getAngle().in(Rotations) - this.setpoint.in(Rotations)) > kEpsilon.in(Rotations);

    PIDOutput = 0;
    if (shouldRun) PIDOutput = BlueMathUtils.clamp(kPID.calculate(getAngle().in(Rotations), this.setpoint.in(Rotations)), -kLimit, kLimit);

    // ! Check PID output before uncommenting
    //if (shouldRun) set(PIDOutput);

    Logger.recordOutput("pivot/angle", getAngle());
    Logger.recordOutput("pivot/setpoint", setpoint);
    Logger.recordOutput("pibot/enabled", enabled);
    Logger.recordOutput("pivot/PIDOutput", PIDOutput);
    SmartDashboard.putData("pivot/PID", kPID);
  }
}

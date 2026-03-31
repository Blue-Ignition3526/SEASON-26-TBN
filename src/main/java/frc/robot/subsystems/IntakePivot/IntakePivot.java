// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakePivot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import lib.BlueShift.math.BlueMathUtils;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.IntakePivot.IntakePivotConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
  private final SparkFlex motor;
  private final CANcoder encoder;

  private Angle setpoint = kPosUp;
  private boolean enabled = true;

  private double PIDOutput;
  private boolean shouldRun;

  public IntakePivot() {
    this.motor = new SparkFlex(kMotorId, MotorType.kBrushless);
    this.encoder = new CANcoder(kEncoderId);

    SparkFlexConfig config = new SparkFlexConfig();
    config
      .smartCurrentLimit(kCurrentLimit)
      .idleMode(IdleMode.kBrake)
      .inverted(true);
  
    this.motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Angle getAngle() {
    return encoder.getPosition().getValue();
  }

  private void setVolts(double voltage) {
    this.motor.setVoltage(voltage);
  }

  public void stop() {
    this.enabled = false;
    this.motor.stopMotor();
  }

  public void setSetpoint(Angle angle) {
    if (angle.in(Rotations) > kMinAngle.in(Rotations) || angle.in(Rotations) < kMaxAngle.in(Rotations)) {
      // In bounds
      if (!enabled) kPID.reset();
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
    return this.setSetpointCommand(kPosUp);
  }

  public Command setUpWaitCommand() {
    return this.setSetpointCommand(kPosUp).alongWith(new WaitUntilCommand(this::atSetpoint));
  }

  public Command setDownCommand() {
    return this.setSetpointCommand(kPosDown);
  }


  public Command setDownWaitCommand() {
    return this.setSetpointCommand(kPosDown).alongWith(new WaitUntilCommand(this::atSetpoint));
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public boolean atSetpoint() {
    return Math.abs(getAngle().in(Rotations) - this.setpoint.in(Rotations)) < kEpsilon.in(Rotations);
  }

  @Override
  public void periodic() {
    shouldRun = this.enabled && !atSetpoint();

    PIDOutput = BlueMathUtils.clamp(kPID.calculate(getAngle().in(Rotations), this.setpoint.in(Rotations)), -kLimit, kLimit);

    if (shouldRun) setVolts(PIDOutput);
    else stop();

    Logger.recordOutput("pivot/angle", getAngle());
    Logger.recordOutput("pivot/setpoint", setpoint);
    Logger.recordOutput("pivot/enabled", enabled);
    Logger.recordOutput("pivot/shouldRun", shouldRun);
    Logger.recordOutput("pivot/atSetpoiny", atSetpoint());
    Logger.recordOutput("pivot/PIDOutput", PIDOutput);
    Logger.recordOutput("pivot/output", motor.getAppliedOutput()*motor.getBusVoltage());
    SmartDashboard.putData("pivot/PID", kPID);
  }
}

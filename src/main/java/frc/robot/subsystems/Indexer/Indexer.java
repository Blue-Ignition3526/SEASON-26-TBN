package frc.robot.subsystems.Indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import static frc.robot.subsystems.Indexer.IndexerConstants.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final SparkFlex wheelMotor;

  public Indexer() {
    this.wheelMotor = new SparkFlex(kWheelMotorID, MotorType.kBrushless);

    SparkFlexConfig topConfig = new SparkFlexConfig();
    
    topConfig
            .smartCurrentLimit(kWheelCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .openLoopRampRate(kWheelRampRate)
            .inverted(false);


    this.wheelMotor.configure(topConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void index() {
    wheelMotor.setVoltage(kTopIndexVoltage);
  }

  public void eject() {
    wheelMotor.setVoltage(kTopEjectVoltage);
  }

  public void stop() {
    wheelMotor.stopMotor();
  }

  public Command indexCommand() {
    return runEnd(this::index, this::stop);
  }

  public Command ejectCommand() {
    return runEnd(this::eject, this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }

  public Command conditionalIndex(BooleanSupplier condition) {
    return runEnd(() -> {
      if(condition.getAsBoolean()) {
        index();
      } else {
        stop();
      }
    }, this::stop);
  }

  public Command conditionalIndexForAuto(BooleanSupplier condition) {
    return run(() -> {
      if(condition.getAsBoolean()) {
        index();
      } else {
        stop();
      }
    });
  }
}

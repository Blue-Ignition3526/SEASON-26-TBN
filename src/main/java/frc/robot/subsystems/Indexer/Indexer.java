package frc.robot.subsystems.Indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.subsystems.Indexer.IndexerConstants.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private final SparkFlex topMotor;
  private final SparkMax lowerMotor;

  public Indexer() {
    this.topMotor = new SparkFlex(kUpperMotorID, MotorType.kBrushless);
    this.lowerMotor = new SparkMax(kLowerMotorID, MotorType.kBrushless);

    SparkFlexConfig upperConfig = new SparkFlexConfig();
    SparkMaxConfig lowerConfig = new SparkMaxConfig();
    
    upperConfig
            .smartCurrentLimit(kUpperCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .openLoopRampRate(kUpperRampRate)
            .inverted(false);

    lowerConfig
            .smartCurrentLimit(kLowerCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .openLoopRampRate(kLowerRampRate)
            .inverted(true);

    this.topMotor.configure(upperConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    this.lowerMotor.configure(lowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void index() {
    topMotor.setVoltage(kTopIndexVoltage);
    lowerMotor.setVoltage(kBottomIndexVoltaje);
  }

  public void eject() {
    topMotor.setVoltage(kTopEjectVoltage);
    lowerMotor.setVoltage(kBottomEjectVoltaje);
  }

  public void stop() {
    topMotor.stopMotor();
    lowerMotor.stopMotor();
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
}

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.subsystems.Shooter.ShooterConstants.*;
import static frc.robot.subsystems.Shooter.ShooterConstants.MagicShooterConstants.*;
import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class ShooterIOMagic implements ShooterIO {
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

  private double tunableSpeed = 0;

  public ShooterIOMagic(Supplier<Pose2d> poseSupplier) {
    leftMotor = new TalonFX(kLeftMotorId, "*");
    s_rightMotor = new TalonFX(kRightMotorID, "*");
    SmartDashboard.putNumber("Shooter/velocity", 0);

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
    setpoint = rps;
    leftMotor.setControl(controlRequest.withVelocity(rps));
  }

  public void standby() {
    set(kIdleSpeed);
  }

  public Command standbyCommand() {
    return run(this::standby);
  }

  public void stop() {
    // This should also stop the follower
    leftMotor.stopMotor();
    automatic = false;
  }

  public double getRPS() {
    return leftMotor.getVelocity().getValueAsDouble();
  }

  public boolean ready() {
    return Math.abs(getRPS() - setpoint) < kEpsilon;
  }

  @Override
  public void calibrate() {
    automatic = false;
    set(tunableSpeed);
  }

  @Override
  public double getSetpoint() { return setpoint; }
  
  public Command setCommand(double rps) {
    return run(()->{automatic = false; set(rps);});
  }

  public Command calibrationCommand() {
    return setCommand(tunableSpeed);
  }

  @Override
  public void shoot() {
    automatic = true;
  }

  public Command shootCommand() {
    return runEnd(()->{automatic=true;}, this::stop);
  }

  public Command stopCommand() {
    return runOnce(() -> {automatic = false; /*this.stop();*/});
  }

  public void configureOrchestra(Orchestra orchestra) {
    orchestra.addInstrument(leftMotor);
    orchestra.addInstrument(s_rightMotor);
  }

  public void periodic() {
    if(automatic) {
      setpoint = TrajectoryGetter.getSetpointWithMap(poseSuppler.get());
      // Check setpoint gives expected values, convert if needed.
      set(setpoint);
    } else {
      // set(0);
      // setpoint = kIdleSpeed;
    }

    tunableSpeed = SmartDashboard.getNumber("Shooter/velocity", 0);
    Logger.recordOutput("Distance", Meters.of(poseSuppler.get().getTranslation().getDistance(TrajectoryGetter.hubPos())));
  }
}

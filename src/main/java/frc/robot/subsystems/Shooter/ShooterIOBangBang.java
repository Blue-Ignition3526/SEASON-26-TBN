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

import static frc.robot.subsystems.Shooter.ShooterConstants.*;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class ShooterIOBangBang implements ShooterIO {
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

    public ShooterIOBangBang(Supplier<Pose2d> poseSupplier) {
        leftMotor = new TalonFX(kLeftMotorId, "*");
        s_rightMotor = new TalonFX(kRightMotorID, "*");

        TalonFXConfiguration leftMotorConfig = new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
        ).withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Amps.of(120))
            .withSupplyCurrentLimit(Amps.of(40))
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
        ).withSlot0(new Slot0Configs()
        // .withKS(kS)
        // .withKV(kV)
        // .withKA(kA)
        // .withKP(kP)
        // .withKI(kI)
        // .withKD(kD)
        );

        TalonFXConfiguration rightMotorConfig = leftMotorConfig.clone();

        leftMotor.getConfigurator().apply(leftMotorConfig);
        s_rightMotor.getConfigurator().apply(rightMotorConfig);

        this.controlRequest = new VelocityVoltage(0);

        leftMotor.setControl(this.controlRequest);
        s_rightMotor.setControl(new Follower(kLeftMotorId, MotorAlignmentValue.Opposed));

        this.poseSuppler = poseSupplier;
    }

    @Override
    public void set(double rps) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'set'");
    }

    @Override
    public void standby() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'standby'");
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public double getRPS() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRPS'");
    }

    @Override
    public boolean ready() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'ready'");
    }

    @Override
    public void configureOrchestra(Orchestra orchestra) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'configureOrchestra'");
    }

    @Override
    public double getSetpoint() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSetpoint'");
    }
    
}

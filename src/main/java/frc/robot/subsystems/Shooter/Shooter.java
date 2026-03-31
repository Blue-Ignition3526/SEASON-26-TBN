package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void set(double rps) { io.set(rps); }
    public void standby() { io.standby(); }
    public void stop() { io.stop(); }
    public double getRPS() { return io.getRPS(); }
    public double getSetpoint() { return io.getSetpoint(); }
    public boolean ready() { return io.ready(); }
    public void configureOrchestra(Orchestra orchestra) { io.configureOrchestra(orchestra); }

    public Command setCommand(double rps) { return runEnd(() -> set(rps), this::stop); };
    public Command standbyCommand() { return runOnce(this::standby); } //* Will be used as default command
    public Command stopCommand() { return runOnce(this::stop); }
    public Command shootCommand() { return runEnd(io::shoot, this::stop); }
    public Command shootCommandForAuto() { return run(io::shoot); }
    public Command calibrationCommand() { return runEnd(io::calibrate, this::stop); }

    @Override
    public void periodic() {
        io.periodic();
        
        Logger.recordOutput("Shooter/isReady", ready());
        Logger.recordOutput("Shooter/Speed", getRPS());
        Logger.recordOutput("Shooter/setpoint", getSetpoint());
    }
}

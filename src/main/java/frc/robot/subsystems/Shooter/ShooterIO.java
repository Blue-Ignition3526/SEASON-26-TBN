package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.Orchestra;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ShooterIO extends Subsystem {
    //? AutoLog

    public void set(double rps);
    default void shoot() {
        set(ShooterConstants.manual3);
    }
    default void calibrate() {
        set(ShooterConstants.manual3);
    }
    public void standby();
    public void stop();
    public double getRPS();
    public double getSetpoint();
    public boolean ready();
    // public Command standbyCommand();
    // public Command setCommand(double rps);
    // public Command shootCommand();
    // public Command stopCommand();
    public void configureOrchestra(Orchestra orchestra);
}

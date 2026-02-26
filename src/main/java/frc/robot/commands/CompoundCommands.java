package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.SwerveChassis.SwerveChassis;

public class CompoundCommands {
  public static Command completeShootCommand(Shooter shooter, Indexer indexer, Hopper hopper, SwerveChassis swerve) {
    return
      shooter.shootCommand()
      .alongWith(hopper.hopperationCommand())
      .alongWith(indexer.conditionalIndex(shooter::ready))
      .alongWith(Commands.runOnce(swerve::xFormation, swerve));
  }
}

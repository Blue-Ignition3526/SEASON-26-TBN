package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.speedAlterators.LookToward;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakePivot.IntakePivot;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.TrajectoryGetter;
import frc.robot.subsystems.SwerveChassis.SwerveChassis;

public class CompoundCommands {
  public static Command completeShootCommand(Shooter shooter, Indexer indexer, Hopper hopper, SwerveChassis swerve) {
    return
      shooter.shootCommand()
        .alongWith(
      hopper.hopperationCommand())
        .alongWith(
      indexer.conditionalIndex(shooter::ready))
      //   .alongWith(
      // Commands.runOnce(swerve::xFormation, swerve))

      .finallyDo(
        () -> {
          shooter.stop();
          hopper.stop();
          indexer.stop();
        }
      );
  }

  public static Command manualShootCommand(Shooter shooter, Indexer indexer, Hopper hopper, double velocity) {
    return 
      shooter.setCommand(velocity)
        .alongWith(
      hopper.hopperationCommand())
        .alongWith(
      indexer.conditionalIndex(shooter::ready))

      .finallyDo(
        () -> {
          shooter.stop();
          hopper.stop();
          indexer.stop();
        }
      );
  }

  // TODO: add intake shake and intake in
  public static Command autoShootCommand(Shooter shooter, Supplier<Pose2d> odoSupplier, Indexer indexer, Hopper hopper, Intake intake, IntakePivot pivot , SwerveChassis swerve) {
    LookToward lookToward = new LookToward(odoSupplier, TrajectoryGetter.hubPos());

    return
      // swerve.enableSpeedAlteratorCommand(lookToward)
      //   .andThen(
      shooter.shootCommandForAuto()
        .alongWith(
      hopper.hopperationCommandForAuto())
        .alongWith(
      indexer.conditionalIndexForAuto(shooter::ready))
        .alongWith(
      intake.setInCommandForAuto())
        .raceWith(
      new WaitCommand(1))
        .andThen(
      new SequentialCommandGroup(
        pivot.setUpWaitCommand(),
        pivot.setDownWaitCommand()
      ).repeatedly()
        .raceWith(
      new WaitCommand(10)))
        .andThen(
      pivot.setUpCommand())

      .finallyDo(
        () -> {
          shooter.stop();
          hopper.stop();
          indexer.stop();
          intake.stop();
          pivot.stop();
        }
      );
  }

  public static Command completeCalibrationCommand(Shooter shooter, Indexer indexer, Hopper hopper) {
    return 
      shooter.calibrationCommand()
        .alongWith(
      hopper.hopperationCommand())
        .alongWith(
      indexer.conditionalIndex(shooter::ready))

      .finallyDo(
        () -> {
          shooter.stop();
          hopper.stop();
          indexer.stop();
        }
      );
  }
}

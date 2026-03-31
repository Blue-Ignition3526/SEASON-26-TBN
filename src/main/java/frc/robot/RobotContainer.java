package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.CompoundCommands;
import frc.robot.commands.DriveSwerve;
import frc.robot.commands.ResetHeadingWithVision;
import frc.robot.speedAlterators.ByHeading;
import frc.robot.speedAlterators.KeepHeading;
import frc.robot.speedAlterators.LookToward;
import frc.robot.speedAlterators.Shake;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.IntakePivot.IntakePivot;
import frc.robot.subsystems.Shooter.ShooterIOMagic;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.TrajectoryGetter;
import frc.robot.subsystems.SwerveChassis.SwerveChassis;
import frc.robot.subsystems.SwerveChassis.SwerveChassisIOReal;
import lib.Elastic;
import lib.Elastic.Notification;
import lib.Elastic.Notification.NotificationLevel;
import lib.BlueShift.control.CustomController;
import lib.BlueShift.control.CustomController.CustomControllerType;
import lib.BlueShift.math.BlueMathUtils;
import lib.BlueShift.odometry.swerve.BlueShiftOdometry;
import lib.BlueShift.odometry.vision.camera.LimelightOdometryCamera;
import lib.BlueShift.odometry.vision.camera.VisionOdometryFilters;

public class RobotContainer {
  // * Controller
  private final CustomController DRIVER = new CustomController(0, CustomControllerType.PS5, 0.09, 1.5, 2.0);
  private final CustomController OPERATOR = new CustomController(1, CustomControllerType.PS5);

  // Swerve Drive
  private final Gyro m_gyro;
  private final SwerveChassis m_swerveChassis;

  private final Indexer indexer;
  private final Hopper hopper;
  private final Intake intake;
  private final IntakePivot intakePivot;
  private final Shooter shooter;

  // * Odometry and Vision
  // private final LimelightOdometryCamera m_limelight3G_Back;
  private final LimelightOdometryCamera m_limelight3G_Front;
  private final BlueShiftOdometry m_odometry;
  private final double m_visionPeriod = 0.02;

  private final LookToward lookTowards;
  private final Shake shake;
  private final ByHeading trenchPass;
  private final KeepHeading keepHeading;

  private final Orchestra orchestra;

  // Speed alterators
  // * Autonomous
  private final SendableChooser<Command> m_autonomousChooser;

  public RobotContainer() {
    //! Subsystems
    // * Swerve Drive
    this.m_gyro = new Gyro(new GyroIOPigeon(Constants.SwerveDriveConstants.kGyroDevice));
    this.m_swerveChassis = new SwerveChassis(new SwerveChassisIOReal(
      new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontLeftOptions),
      new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontRightOptions),
      new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackLeftOptions),
      new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackRightOptions),
      m_gyro
    ));

    this.indexer = new Indexer();
    this.hopper = new Hopper();
    this.intake = new Intake();
    this.intakePivot = new IntakePivot();

    // ! Odometry and Vision
    // this.m_limelight3G_Back = new LimelightOdometryCamera(Constants.Vision.Limelight3G_Back.kName, true, true, VisionOdometryFilters::visionFilter);
    this.m_limelight3G_Front = new LimelightOdometryCamera(Constants.Vision.Limelight3G_Front.kName, true, true, VisionOdometryFilters::visionFilter);
    this.m_odometry = new BlueShiftOdometry(
      Constants.SwerveDriveConstants.PhysicalModel.kDriveKinematics,
      m_swerveChassis::getHeading,
      m_swerveChassis::getModulePositions,
      new Pose2d(),
      m_visionPeriod,
      // m_limelight3G_Back,
      m_limelight3G_Front
    );
    this.m_limelight3G_Front.enable();
    this.m_odometry.startVision();

    this.shooter = new Shooter(new ShooterIOMagic(m_odometry::getEstimatedPosition));

    this.lookTowards = new LookToward(m_odometry::getEstimatedPosition, TrajectoryGetter.hubPos());
    this.shake = new Shake(Rotation2d.fromDegrees(1), m_swerveChassis::getHeading);
    this.trenchPass = new ByHeading(() -> m_swerveChassis.getHeading().getRadians(), () -> BlueMathUtils.optimize(m_gyro.getHeading(), Rotation2d.kZero).getRadians());
    this.keepHeading = new KeepHeading(m_odometry::getEstimatedPosition, 0.05);

    this.m_swerveChassis.setDefaultAlterator(keepHeading);

    NamedCommands.registerCommand("SHOOT", CompoundCommands.autoShootCommand(shooter, m_odometry::getEstimatedPosition, indexer, hopper, intake, intakePivot, m_swerveChassis));
    NamedCommands.registerCommand("INTAKE_DOWN", intakePivot.setDownCommand());
    NamedCommands.registerCommand("INTAKE_UP", intakePivot.setUpCommand());
    NamedCommands.registerCommand("INTAKE_START", intake.runOnce(intake::setIn));
    NamedCommands.registerCommand("INTAKE_STOP", intake.stopCommand());
    NamedCommands.registerCommand("AUTOAIM", m_swerveChassis.enableSpeedAlteratorCommand(lookTowards));
    NamedCommands.registerCommand("STOP_ALTERATORS", m_swerveChassis.disableSpeedAlteratorCommand());

    // ! Speed alterators
    // this.m_speedAlterator_turn180 = new Turn180(m_odometry::getEstimatedPosition);
    // this.m_speedAlterator_lookAt = new LookController(this.m_swerveDrive::getHeading, this.DRIVER::getRightX, this.DRIVER::getRightY, Constants.SwerveDriveConstants.kJoystickDeadband);
    // this.m_speedAlterator_AlignToNearestBranch = new AlignToNearestBranch(m_odometry::getEstimatedPosition, this.DRIVER.rightBumper()::getAsBoolean, this.DRIVER::getLeftY, this.DRIVER::getLeftX);
    
    // ! Autonomous
    // Robot config
    RobotConfig ppRobotConfig = null;
    try{
      ppRobotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "ERROR! COULD NOT LOAD PP ROBOT CONFIG", e.getMessage()));
      DriverStation.reportError("ERROR! COULD NOT LOAD PP ROBOT CONFIG", e.getStackTrace());
    }

    AutoBuilder.configure(
      m_odometry::getEstimatedPosition,
      m_odometry::resetPosition,
      m_swerveChassis::getRobotRelativeChassisSpeeds,
      (ChassisSpeeds speeds, DriveFeedforwards ff) -> m_swerveChassis.driveRobotRelative(speeds),
      new PPHolonomicDriveController(
        SwerveDriveConstants.AutonomousConstants.kTranslatePIDConstants,
        SwerveDriveConstants.AutonomousConstants.kRotatePIDConstants
      ),
      ppRobotConfig,
      () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
      m_swerveChassis
    );

    // Build auto chooser
    this.m_autonomousChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("AutoChooser", m_autonomousChooser);    
    // ! Dashboard testing commands
    // Chassis
    SmartDashboard.putData("SwerveDrive/ResetTurningEncoders", new InstantCommand(m_swerveChassis::resetTurningEncoders).ignoringDisable(true));
    SmartDashboard.putData("SwerveDrive/ResetHeadingWithVision", new ResetHeadingWithVision(m_gyro, Constants.Vision.Limelight3G_Front.kName));

    this.orchestra = new Orchestra();

    shooter.configureOrchestra(orchestra);
    m_swerveChassis.configureOrchestra(orchestra);

    // ! Add controller bindings
    configureBindings();
  }

  private void configureBindings() {
    // ! DRIVER BINDINGS
    // https://www.gameuidatabase.com/tb_pad/index.php?templates=Controller+Scheme+1&leftTrigger=Turn+Left&rightTrigger=Turn+Right&rightBumper=Robot+Relative+%28Hold%29&leftStick=Movement&rightStickClick=Set+Zero+Heading&xButton=Outake&bButton=Shoot&aButton=Intake&col=%231A53AE%2C%231A313A%2C%23FFFFFF&startButton=Self+Destruct&leftBumper=Auto+Aim
    // * Swerve drive binding
    this.m_swerveChassis.setDefaultCommand(new DriveSwerve(
        m_swerveChassis,
        () -> -DRIVER.getLeftY(),
        () -> -DRIVER.getLeftX(),
        () ->  (DRIVER.getLeftTrigger() - DRIVER.getRightTrigger()),
        () -> true // /!DRIVER.rightBumper().getAsBoolean()
      )
    );

    this.indexer.setDefaultCommand(indexer.ejectCommand());
    this.hopper.setDefaultCommand(hopper.reverseCommand());

    // shooter.setDefaultCommand(shooter.standbyCommand());

    // * Reset heading with right stick button
    this.DRIVER.rightStickButton().onTrue(this.m_swerveChassis.zeroHeadingCommand());

    DRIVER.leftBumper().onTrue(m_swerveChassis.enableSpeedAlteratorCommand(lookTowards)).onFalse(m_swerveChassis.disableSpeedAlteratorCommand());
    DRIVER.rightBumper().onTrue(m_swerveChassis.enableSpeedAlteratorCommand(trenchPass)).onFalse(m_swerveChassis.disableSpeedAlteratorCommand());

    DRIVER.rightButton().whileTrue(CompoundCommands.completeShootCommand(shooter, indexer, hopper, m_swerveChassis))
      .onFalse(shooter.standbyCommand());
    // DRIVER.bottomButton().whileTrue(intake.setInCommand());
    // DRIVER.topButton().whileTrue(intake.setOutCommand());

    // DRIVER.rightButton().whileTrue(CompoundCommands.manualShootCommand(shooter, indexer, hopper, ShooterConstants.manual3));
    DRIVER.bottomButton().whileTrue(intake.setInCommand());

    // Self Destruct Commandn
    DRIVER.startButton().onTrue(Commands.runOnce(() -> { orchestra.loadMusic("rickroll.chrp"); orchestra.play(); }).ignoringDisable(true));
    OPERATOR.startButton().onTrue(Commands.runOnce(() -> { orchestra.loadMusic("megalovania.chrp"); orchestra.play(); }).ignoringDisable(true));

    SmartDashboard.putData("Sing/rick", Commands.runOnce(() -> { orchestra.loadMusic("rickroll.chrp"); orchestra.play(); }).ignoringDisable(true));
    SmartDashboard.putData("Sing/megalovania", Commands.runOnce(() -> { orchestra.loadMusic("megalovania.chrp"); orchestra.play(); }).ignoringDisable(true));
    SmartDashboard.putData("Sing/topgun", Commands.runOnce(() -> { orchestra.loadMusic("topgun.chrp"); orchestra.play(); }).ignoringDisable(true));

    // ! OPERATOR
    OPERATOR.topButton().whileTrue(CompoundCommands.manualShootCommand(shooter, indexer, hopper, ShooterConstants.manual4));
    // OPERATOR.rightButton().whileTrue(CompoundCommands.manualShootCommand(shooter, indexer, hopper, ShooterConstants.manual2));
    OPERATOR.rightButton().whileTrue(CompoundCommands.completeShootCommand(shooter, indexer, hopper, m_swerveChassis));
    OPERATOR.leftButton().whileTrue(CompoundCommands.manualShootCommand(shooter, indexer, hopper, ShooterConstants.manual3));
    OPERATOR.bottomButton().whileTrue(CompoundCommands.completeCalibrationCommand(shooter, indexer, hopper));

    OPERATOR.rightBumper().onTrue(intakePivot.setDownCommand());
    OPERATOR.leftBumper().onTrue(intakePivot.setUpCommand());

    OPERATOR.leftTrigger().whileTrue(intake.setInCommand());
    OPERATOR.rightTrigger().whileTrue(intake.setOutCommand());

    OPERATOR.rightStickButton()
      .onTrue(m_swerveChassis.enableSpeedAlteratorCommand(shake))
      .onFalse(m_swerveChassis.disableSpeedAlteratorCommand());

    OPERATOR.leftStickButton()
      .whileTrue(
        Commands.run(
          m_swerveChassis::xFormation,
          m_swerveChassis
        )
      );

    OPERATOR.povRight().whileTrue(indexer.indexCommand());
    // OPERATOR.povRight().whileTrue(hopper.hopperationCommand());
    OPERATOR.povLeft().onTrue(CompoundCommands.autoShootCommand(shooter, m_odometry::getEstimatedPosition, indexer, hopper, intake, intakePivot, m_swerveChassis));
    OPERATOR.povUp().whileTrue(indexer.ejectCommand().alongWith(hopper.reverseCommand()));
    // DRIVER.povUp().whileTrue(indexer.ejectCommand().alongWith(hopper.reverseCommand()));
    OPERATOR.povDown().whileTrue(CompoundCommands.completeCalibrationCommand(shooter, indexer, hopper));
  }

  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected().finallyDo(m_swerveChassis::disableSpeedAlterator);
  }
}

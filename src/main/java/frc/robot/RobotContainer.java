package frc.robot;

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.autos.Autos;
import frc.robot.commands.CompoundCommands;
import frc.robot.commands.DriveSwerve;
import frc.robot.speedAlterators.LookToward;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import frc.robot.subsystems.Hopper.Hopper;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.SwerveChassis.SwerveChassis;
import frc.robot.subsystems.SwerveChassis.SwerveChassisIOReal;
import frc.robot.subsystems.SwerveChassis.SwerveChassisIOSim;
import lib.Elastic;
import lib.Elastic.Notification;
import lib.Elastic.Notification.NotificationLevel;
import lib.BlueShift.control.CustomController;
import lib.BlueShift.control.SpeedAlterator;
import lib.BlueShift.control.CustomController.CustomControllerType;
import lib.BlueShift.odometry.swerve.BlueShiftOdometry;
import lib.BlueShift.odometry.vision.camera.LimelightOdometryCamera;
import lib.BlueShift.odometry.vision.camera.VisionOdometryFilters;

public class RobotContainer {
  // * Controllers
  private final CustomController DRIVER = new CustomController(0, Robot.isReal() ? CustomControllerType.XBOX : CustomControllerType.PS5);
  private final CustomController OPERATOR = new CustomController(1, CustomControllerType.XBOX);

  // Swerve Drive
  private final SwerveChassis m_swerveChassis;

  private final Indexer indexer;
  private final Hopper hopper;
  private final Intake intake;
  private final Shooter shooter;

  // * Odometry and Vision
  // private final LimelightOdometryCamera m_limelight3G_Back;
  private final LimelightOdometryCamera m_limelight3G_Front;
  private final BlueShiftOdometry m_odometry;
  private final double m_visionPeriod = 0.02;

  private final SpeedAlterator lookTowards;

  private final Orchestra orchestra;

  // Speed alterators
  // * Autonomous
  private final SendableChooser<Command> m_autonomousChooser;

  public RobotContainer() {
    //! Subsystems
    // * Swerve Drive
    if (Robot.isReal()) {
      this.m_swerveChassis = new SwerveChassis(new SwerveChassisIOReal(
        new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontLeftOptions),
        new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontRightOptions),
        new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackLeftOptions),
        new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackRightOptions),
        new Gyro(new GyroIOPigeon(Constants.SwerveDriveConstants.kGyroDevice))
      ));
    } else {
      this.m_swerveChassis = new SwerveChassis(new SwerveChassisIOSim());
    }

    this.indexer = new Indexer();
    this.hopper = new Hopper();
    this.intake = new Intake();

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

    this.shooter = new Shooter(m_odometry::getEstimatedPosition);

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
    this.m_autonomousChooser = new SendableChooser<Command>();

    try {
      m_autonomousChooser.addOption("TestAuto", Autos.testAuto());
    } catch (Exception e) {
      DriverStation.reportError("Error building autos", e.getStackTrace());
      Elastic.sendNotification(new Notification(NotificationLevel.ERROR, "Error building autos", e.getMessage()));
    }

    SmartDashboard.putData("AutoChooser", m_autonomousChooser);

    this.lookTowards = new LookToward(m_odometry::getEstimatedPosition, FieldConstants.kHubPosition);
    
    // ! Dashboard testing commands
    // Chassis
    SmartDashboard.putData("SwerveDrive/ResetTurningEncoders", new InstantCommand(m_swerveChassis::resetTurningEncoders).ignoringDisable(true));

    this.orchestra = new Orchestra();
    
    shooter.configureOrchestra(orchestra);
    m_swerveChassis.configureOrchestra(orchestra);

    orchestra.loadMusic("filepath");

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
        () -> DRIVER.getLeftTrigger() - DRIVER.getRightTrigger(),
        () -> !DRIVER.bottomButton().getAsBoolean()
      )
    );

    // * Reset heading with right stick button
    //TODO: think of a better button to bind this to
    this.DRIVER.rightStickButton().onTrue(this.m_swerveChassis.zeroHeadingCommand());

    DRIVER.rightButton().onTrue(m_swerveChassis.enableSpeedAlteratorCommand(lookTowards)).onFalse(m_swerveChassis.disableSpeedAlteratorCommand());
    DRIVER.leftButton().onTrue(CompoundCommands.completeShootCommand(shooter, indexer, hopper, m_swerveChassis));

    // Self Destruct Command
    DRIVER.startButton().onTrue(Commands.runOnce(orchestra::play).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }
}

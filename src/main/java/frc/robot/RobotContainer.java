package frc.robot;

import org.opencv.core.Point;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.commands.DriveSwerve;
import frc.robot.speedAlterators.LookToward;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import frc.robot.subsystems.SwerveDrive.SwerveDrive;
import frc.robot.subsystems.SwerveDrive.SwerveDriveIOReal;
import frc.robot.subsystems.SwerveDrive.SwerveDriveIOSim;
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
  private final SwerveDrive m_swerveDrive;

  // Speed alterators

  // * Odometry and Vision
  private final LimelightOdometryCamera m_limelight3G_Back;
  private final LimelightOdometryCamera m_limelight3G_Front;
  private final BlueShiftOdometry m_odometry;
  private final double m_visionPeriod = 0.02;

  private final SpeedAlterator lookTowards;

  // * Autonomous
  private final SendableChooser<Command> m_autonomousChooser;

  public RobotContainer() {
    //! Subsystems
    // * Swerve Drive
    if (Robot.isReal()) {
      this.m_swerveDrive = new SwerveDrive(new SwerveDriveIOReal(
        new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontLeftOptions),
        new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kFrontRightOptions),
        new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackLeftOptions),
        new SwerveModule(Constants.SwerveDriveConstants.SwerveModuleConstants.kBackRightOptions),
        new Gyro(new GyroIOPigeon(Constants.SwerveDriveConstants.kGyroDevice))
      ));
    } else {
      this.m_swerveDrive = new SwerveDrive(new SwerveDriveIOSim());
    }

    // ! Odometry and Vision
    this.m_limelight3G_Back = new LimelightOdometryCamera(Constants.Vision.Limelight3G_Back.kName, true, true, VisionOdometryFilters::visionFilter);
    this.m_limelight3G_Front = new LimelightOdometryCamera(Constants.Vision.Limelight3G_Front.kName, true, true, VisionOdometryFilters::visionFilter);
    this.m_odometry = new BlueShiftOdometry(
      Constants.SwerveDriveConstants.PhysicalModel.kDriveKinematics,
      m_swerveDrive::getHeading,
      m_swerveDrive::getModulePositions,
      new Pose2d(),
      m_visionPeriod,
      m_limelight3G_Back,
      m_limelight3G_Front
    );
    this.m_limelight3G_Back.enable();
    this.m_limelight3G_Front.enable();
    this.m_odometry.startVision();

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
      m_swerveDrive::getRobotRelativeChassisSpeeds,
      (ChassisSpeeds speeds, DriveFeedforwards ff) -> m_swerveDrive.driveRobotRelative(speeds),
      new PPHolonomicDriveController(
        SwerveDriveConstants.AutonomousConstants.kTranslatePIDConstants,
        SwerveDriveConstants.AutonomousConstants.kRotatePIDConstants
      ),
      ppRobotConfig,
      () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
      m_swerveDrive
    );

    // Build auto chooser
    this.m_autonomousChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("AutoChooser", m_autonomousChooser);

    this.lookTowards = new LookToward(m_odometry::getEstimatedPosition, new Point(4.62, 4.03));
    
    // ! Dashboard testing commands
    // Chassis
    SmartDashboard.putData("SwerveDrive/ResetTurningEncoders", new InstantCommand(m_swerveDrive::resetTurningEncoders).ignoringDisable(true));

    // ! Add controller bindings
    configureBindings();
  }

  private void configureBindings() {
    // ! DRIVER BINDINGS
    // * Swerve drive binding
    this.m_swerveDrive.setDefaultCommand(new DriveSwerve(
        m_swerveDrive,
        () -> -DRIVER.getLeftY(),
        () -> -DRIVER.getLeftX(),
        () -> DRIVER.getLeftTrigger() - DRIVER.getRightTrigger(),
        () -> !DRIVER.bottomButton().getAsBoolean()
      )
    );

    // * Reset heading with right stick button
    //TODO: think of a better button to bind this to
    this.DRIVER.rightStickButton().onTrue(this.m_swerveDrive.zeroHeadingCommand());

    DRIVER.rightButton().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(lookTowards)).onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
  }

  public Command getAutonomousCommand() {
    return m_autonomousChooser.getSelected();
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.SwerveChassis.SwerveChassis;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ResetHeadingWithVision extends Command {
  SwerveChassis m_chassis;
  Gyro m_gyro;
  String m_llName;

  public ResetHeadingWithVision(/* SwerveChassis chassis,*/ Gyro gyro, String llName) {
    /*this.m_chassis = chassis;*/
    this.m_gyro = gyro;
    this.m_llName = llName;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightHelpers.PoseEstimate res = LimelightHelpers.getBotPoseEstimate_wpiBlue(m_llName);
    m_gyro.reset(res.pose.getRotation());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

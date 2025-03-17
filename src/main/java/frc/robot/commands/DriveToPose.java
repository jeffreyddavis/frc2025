// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;
import frc.robot.addons.LimelightHelpers;
import frc.robot.addons.PIDMint;
import frc.robot.subsystems.drive.Drive;

public class DriveToPose extends Command {
  private PIDMint xController, yController, rotController;
  private Timer dontSeeTagTimer, stopTimer;
  private Drive drive;
  private boolean m_extraTolerance;
  @AutoLogOutput
  private Pose2d m_targetPose2d;
  private CommandJoystick m_driverController;

  public DriveToPose(Pose2d targetPose2d, Drive drive, CommandJoystick driverController, boolean extraTolerance) {
    m_targetPose2d = targetPose2d;
    m_driverController = driverController;
    m_extraTolerance = extraTolerance;
    xController = new PIDMint(Constants.Limelight.X_REEF_ALIGNMENT_P, 1.1, 0, Constants.Limelight.X_TOLERANCE_REEF_ALIGNMENT, 0);  // Vertical movement
    yController = new PIDMint(Constants.Limelight.Y_REEF_ALIGNMENT_P, 1.1, 0, Constants.Limelight.Y_TOLERANCE_REEF_ALIGNMENT, 0);  // Horitontal movement
    rotController = new PIDMint(Constants.Limelight.ROT_REEF_ALIGNMENT_P, .03, 0, Constants.Limelight.ROT_TOLERANCE_REEF_ALIGNMENT, 0);  // Rotation
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer(); 
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(m_targetPose2d.getRotation().getDegrees());
    
    SmartDashboard.putNumber("Target Rot", m_targetPose2d.getRotation().getDegrees());
    if (m_extraTolerance) rotController.setTolerance(Constants.Limelight.ROT_TOLERANCE_REEF_ALIGNMENT * 3);
    else rotController.setTolerance(Constants.Limelight.ROT_TOLERANCE_REEF_ALIGNMENT);
    rotController.enableContinuousInput(-180, 180);
 
    xController.setSetpoint(m_targetPose2d.getX());
    SmartDashboard.putNumber("Target X", m_targetPose2d.getX());
    if (m_extraTolerance) xController.setTolerance(Constants.Limelight.X_TOLERANCE_REEF_ALIGNMENT * 3);
    else xController.setTolerance(Constants.Limelight.X_TOLERANCE_REEF_ALIGNMENT);


    
    SmartDashboard.putNumber("Target Y", m_targetPose2d.getY());
    yController.setSetpoint(m_targetPose2d.getY());

    if (m_extraTolerance) yController.setTolerance(Constants.Limelight.Y_TOLERANCE_REEF_ALIGNMENT*3);
    else yController.setTolerance(Constants.Limelight.Y_TOLERANCE_REEF_ALIGNMENT);

    
    SmartDashboard.putString("Ended", "nope");
  }

  @Override
  public void execute() {
      
      this.dontSeeTagTimer.reset();

      Pose2d current = drive.getPose();

      SmartDashboard.putNumber("Current X", current.getX());
      double xSpeed = xController.calculate(current.getX());
      SmartDashboard.putNumber("xspeed", xSpeed);
      
      SmartDashboard.putNumber("Current Y", current.getY());
      double ySpeed = -yController.calculate(current.getY());
      SmartDashboard.putNumber("yspeed", ySpeed);
      double rotValue = rotController.calculate(current.getRotation().getDegrees());
      
      SmartDashboard.putNumber("Current rot", current.getRotation().getDegrees());
      SmartDashboard.putNumber("rotspeed", rotValue);

      xSpeed -= m_driverController.getRawAxis(1);
      ySpeed -= m_driverController.getRawAxis(0);
      rotValue -= m_driverController.getRawAxis(2);

      drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, -ySpeed, rotValue, drive.getPose().getRotation()));

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    
    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Ended", "yep");
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return 
        stopTimer.hasElapsed(Constants.Limelight.POSE_VALIDATION_TIME) 
        || Math.abs(m_driverController.getRawAxis(0)) > .5
        || Math.abs(m_driverController.getRawAxis(1)) > .5
        || Math.abs(m_driverController.getRawAxis(2)) > .5
        || m_driverController.button(1).getAsBoolean()
        ;
  }
         
}
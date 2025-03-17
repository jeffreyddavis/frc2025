// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.addons.LimelightHelpers;
import frc.robot.subsystems.drive.Drive;
import frc.robot.Constants;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private Drive drive;
  private double tagID = -1;

  public AlignToReefTagRelative(boolean isRightScore, Drive drive) {
    xController = new PIDController(Constants.Limelight.X_REEF_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    yController = new PIDController(Constants.Limelight.Y_REEF_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    rotController = new PIDController(Constants.Limelight.ROT_REEF_ALIGNMENT_P, 0, 0);  // Rotation
    this.isRightScore = isRightScore;
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    rotController.setSetpoint(Constants.Limelight.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.Limelight.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.Limelight.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.Limelight.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(isRightScore ? Constants.Limelight.Y_SETPOINT_REEF_ALIGNMENT : -Constants.Limelight.Y_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.Limelight.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight-reefr");
    
    SmartDashboard.putString("started", "yep");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("limelight-reefr") && LimelightHelpers.getFiducialID("limelight-reefr") == tagID) {
      
    SmartDashboard.putString("Sees Tag", "yep");
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-reefr");
      SmartDashboard.putNumber("x", postions[2]);

      SmartDashboard.putNumber("y", postions[4]);


      double xSpeed = xController.calculate(postions[2]);
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = -yController.calculate(postions[0]);
      SmartDashboard.putNumber("yspeed", xSpeed);
      double rotValue = -rotController.calculate(postions[4]);
      SmartDashboard.putNumber("rotspeed", xSpeed);

      drive.runVelocity(new ChassisSpeeds(-xSpeed, -ySpeed, rotValue));

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      
    SmartDashboard.putString("Sees Tag", "nope");
      drive.runVelocity(new ChassisSpeeds(0, 0, 0));
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
    return this.dontSeeTagTimer.hasElapsed(Constants.Limelight.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.Limelight.POSE_VALIDATION_TIME);
  }
         
}
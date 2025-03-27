// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

/**
 * /** Patient reminder to revisit the calculations on this file after QA Testing.
 *
 * <p>Thanks, - Jeff (the shorter one)
 */
public final class Constants {

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kTestingControllerPort = 1;
  }


  public static class controller {
    public static final int ShootButton = 1;
    public static final int EjectButton = 11;
    public static final int GoToL1 = 10;
    public static final int GoToL2 = 5;
    public static final int GoToL3 = 6;
    public static final int GoToL4 = 7;
    public static final int ThrowAlgaeTheButton = 14;
  }

  public static class Arm {
    //public static final int rightMotor = 6;
    public static final int leftMotor = 7;

    public static final int Encoder = 0;
    public static final double angleTolerance = 3;
    public static final double gravityFF = .025;

    public static final double armSpeed = .05;
    public static final double rampUpTime = .5;
    public static final double testSpeed = 0.2;
    public static final double holdSpeed = .03;
    public static final double encoderOffset = .75;

    public static final double ThrowingAngle = 160;
    public static final double WindUpAngle = 30;

    public static final double ScoringAngle = 138;
    public static final double HighScoringAngle = 160;
    //public static final double intakeAngle = 175;
    public static final double intakeAngle  = 190;
    
    public static final double startingPosition = 172;
    public static final double StraightOut = 85;
    public static final double Processor = 112;
    public static final double GetAlgaeAngle = 115;
    public static final double CarryAngle = 162;
    public static final double SafeCarryAngle = 145;
    public static final double Max = 190;
  }

  public static class Intake {
    public static final int Motor = 10;

    public static final double intakeAlgaeSpeed = 1;
    public static final double shootAlgaeSpeed = -1;
    public static final double shootCoralSpeed = -.35;
    public static final double shootCoralSpeedAuto = -.7;
    public static final double testSpeed = 1;
    public static final double holdSpeed = .15;
    public static final int intakeLimitSwitchId = 4;
    public static final int intakeLimitSwitch2Id = 5;
  }

  public static class Limelight {
  public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;
  public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 2;
  public static final double Y_REEF_ALIGNMENT_P = 2;
  public static final double Y_SETPOINT_REEF_ALIGNMENT = 8;
  public static final double Y_TOLERANCE_REEF_ALIGNMENT = .021;
  public static final double ROT_REEF_ALIGNMENT_P = .08;
  public static final double X_SETPOINT_REEF_ALIGNMENT = -20;
  public static final double X_TOLERANCE_REEF_ALIGNMENT = .021;
  public static final double X_REEF_ALIGNMENT_P = 2;
  public static final double DONT_SEE_TAG_WAIT_TIME = .1;
  public static final double POSE_VALIDATION_TIME = .1;
  }

  public static class Elevator {
    public static final int leftMotor = 2;
    public static final int rightMotor = 3;

    public static final int elevatorEncoder = 1;
    public static final double heightTolerance = 50;

    public static final double elevatorSpeed = .5;
    public static final double testSpeed = 1;
    public static final double gravityFF = .055;

    public static final double StaticHeight = 0;
    public static final double IntakeHeight = 200;
    //public static final double IntakeHeight = 1000;
    public static final double ArmClearHeight = 10;
    public static final double ProcessorHeight = 300;
    public static final double SeaFloorHeight = 200;
    public static final double DunkDistance = 500;
    public static final double HeightL1 = 250;
    public static final double HeightL2 = 2700;
    public static final double HeightL3 = 6100;
    public static final double HeightL4 = 9800;
    public static final double HeightNET = 9600;
    public static final double MaxSafeHeight = 3200;
    public static final double MinSafeHeight = 4200;
    public static final double StowHeight = 2000;
    public static final double HeightAlgaeLow = 3200;
    public static final double HeightAlgaeHi = 6800;


    public static final double maxHeight = 4.3;
  }

  /*
  * Our climber system has been physically changed, so we may need to adjust.
  *
  * Or not. That would likely be unnecessary. I don't know.
  * I'm just trying to make myself write comments so I can use them when I actually need to.
  *
  * - Jeff (the smaller one)
  /*/

  public static class Climber {
    public static final int leftMotor = 23;
    // public static final int rightMotor = 23;

    public static final int Encoder = 6;
    public static final int otherencoder = 7;
    public static final double Tolerance = 200;

    public static final double Speed = 1;
    public static final double rampUpTime = 2;
    public static final double testSpeed = 1;
    public static final double holdSpeed = .03;
    public static final double encoderOffset = 0;

    public static final double DownHeight = 0;
    public static final double TopHeight = 7000;
  }

  public static class Hopper {
    public static final int leftMotor = 20;
    public static final int rightMotor = 21;
    public static final double testSpeed = .20;
    public static final double Speed = .1;
    public static final double holdSpeed = .1;
  }

  public static class VisionTargeting {
    
  }
}

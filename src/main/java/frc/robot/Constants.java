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

  public static class PhotonCamera {
    public static final Transform3d cameraOffset = new Transform3d(50, 50, 50, new Rotation3d());
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
    public static final double angleTolerance = .5;

    public static final double armSpeed = .05;
    public static final double rampUpTime = 0;
    public static final double testSpeed = 0.2;
    public static final double holdSpeed = .03;
    public static final double encoderOffset = .41;

    public static final double DownAngle = 0;
    public static final double ScoringAngle = 135;
    public static final double ThrowingAngle = 160;
    public static final double startingPosition = 168.5;
    public static final double ReleasingAngle = 135;
    public static final double StraightOut = 80;
    public static final double Max = 165;
  }

  public static class Intake {
    public static final int Motor = 10;

    public static final double intakeAlgaeSpeed = 1;
    public static final double shootAlgaeSpeed = -1;
    public static final double shootCoralSpeed = -.1;
    public static final double testSpeed = 1;
    public static final double holdSpeed = .1;
    public static final int intakeLimitSwitchId = 0;
    public static final int intakeLimitSwitch2Id = 1;
  }

  public static class Elevator {
    public static final int leftMotor = 2;
    public static final int rightMotor = 3;

    public static final int elevatorEncoder = 1;
    public static final double heightTolerance = .5;
    public static final double encoderOffset =
        -.0394; // This offset consistently appears every init.
    public static final double distanceMultiple =
        100; // Give us distances in easier to read whole numbers

    public static final double elevatorSpeed = .3;
    public static final double testSpeed = .5;

    public static final double StaticHeight = 0;
    public static final double ArmClearHeight = 10;
    public static final double ProcessorHeight = 300;
    public static final double SeaFloorHeight = 0;
    public static final double DunkDistance = 10;
    public static final double HeightL1 = 200;
    public static final double HeightL2 = 60;
    public static final double HeightL3 = 90;
    public static final double HeightL4 = 120;
    public static final double HeightNET = 150;
    public static final double HeightAlgaeLow = 150;
    public static final double HeightAlgaeHi = 500;


    public static final double maxHeight = 4.3;
  }

  public static class Climber {
    //public static final int leftMotor = 22;
    public static final int rightMotor = 23;

    public static final int Encoder = 0;
    public static final double angleTolerance = .5;

    public static final double Speed = .05;
    public static final double rampUpTime = 0;
    public static final double testSpeed = .50;
    public static final double holdSpeed = .03;
    public static final double encoderOffset = .363;

    public static final double DownAngle = 0;
    public static final double ScoringAngle = 135;
    public static final double ThrowingAngle = 160;
    public static final double ReleasingAngle = 135;
    public static final double StraightOut = 80;
    public static final double Max = 165;
  }

  public static class Hopper {
    public static final int leftMotor = 20;
    public static final int rightMotor = 21;
    public static final double testSpeed = .20;
    public static final double Speed = .1;
    public static final double holdSpeed = .1;
  }
}

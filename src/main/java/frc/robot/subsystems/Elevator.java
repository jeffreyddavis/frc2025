// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.addons.PIDMint;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public PIDMint ElevatorControl;

  public SparkMax leftMotor;
  public SparkMax rightMotor;
  public SparkMaxConfig leftMotorConfig;
  public SparkMaxConfig rightMotorConfig;
  public Encoder elevatorEncoder;

  @AutoLogOutput
  private double targetHeight = 0;
  @AutoLogOutput
  private boolean movingToTarget = false;
  @AutoLogOutput
  private boolean MaxSpeedMode = false;

  public enum Targets {
    Floor,
    Processor,
    Intake,
    L1,
    L2,
    L3,
    L4,
    AlgaeLow,
    AlgaeHi,
    NET
  }

  public Elevator() {
    leftMotor =
        new SparkMax(
            Constants.Elevator.leftMotor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    rightMotor =
        new SparkMax(
            Constants.Elevator.rightMotor,
            com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    ElevatorControl = new PIDMint(0.00033, 0, .0000001, 50, .08);
    leftMotorConfig = new SparkMaxConfig();
    leftMotorConfig.inverted(false);
    leftMotorConfig.idleMode(IdleMode.kBrake);
    leftMotorConfig.openLoopRampRate(0);

    rightMotorConfig = new SparkMaxConfig();
    rightMotorConfig.inverted(true);
    rightMotorConfig.follow(leftMotor, true);
    //rightMotorConfig.disableFollowerMode();
    rightMotorConfig.idleMode(IdleMode.kBrake);
    rightMotorConfig.openLoopRampRate(0);


    rightMotor.configure(rightMotorConfig, null, null);
    leftMotor.configure(leftMotorConfig, null, null);

    elevatorEncoder = new Encoder(1, 2);

  }

  public void setMaxSpeedMode(boolean max) {
    this.MaxSpeedMode = max;
  }

  @AutoLogOutput
  public boolean isAtLocation() {
    return (Math.abs(getLocation() - targetHeight) < Constants.Elevator.heightTolerance);
  }
  public boolean isNearLocation() {
    return (Math.abs(getLocation() - targetHeight) < (Constants.Elevator.heightTolerance * 4));
  }

  public boolean GoToTarget(Elevator.Targets target) {
    double targetHeight = 0; 
    switch (target) {
      case Floor:
        targetHeight = Constants.Elevator.SeaFloorHeight;
        break;
      case Processor:
        targetHeight = Constants.Elevator.ProcessorHeight;
        break;
      case L1:
        targetHeight = Constants.Elevator.HeightL1;
        break;
      case L2:
        targetHeight = Constants.Elevator.HeightL2;
        break;
      case L3:
        targetHeight = Constants.Elevator.HeightL3;
        break;
      case L4:
        targetHeight = Constants.Elevator.HeightL4;
        break;
      case NET:
        targetHeight = Constants.Elevator.HeightNET;
        break;
      case AlgaeHi:
        targetHeight = Constants.Elevator.HeightAlgaeHi;
        break;
      case AlgaeLow:
        targetHeight = Constants.Elevator.HeightAlgaeLow;
        break;
      case Intake:
        targetHeight = Constants.Elevator.IntakeHeight;
        break;
      default:
        targetHeight = Constants.Elevator.SeaFloorHeight;
    }
    return goToLocation(targetHeight);
  }

  public boolean goToLocation(double targetHeight) {
    this.targetHeight = targetHeight;
    this.movingToTarget = true;
    return isAtLocation();
  }

  @AutoLogOutput
  public double getLocation() {
    return -elevatorEncoder.getDistance();
  }

  @AutoLogOutput
  public boolean armClear() {
    return getLocation() >= Constants.Elevator.ArmClearHeight;
  }

  public void stop() {
    this.movingToTarget = false;
    leftMotor.set(Constants.Elevator.gravityFF);
    //leftMotor.stopMotor();
    //rightMotor.stopMotor();
  }

  public void dunk() {
    this.targetHeight = getLocation() - Constants.Elevator.DunkDistance;
    this.movingToTarget = true;
  }

  public void DunkAuto() {
    this.targetHeight = getLocation() - Constants.Elevator.DunkDistance * 6;
    this.movingToTarget = true;
  }

  public void testUp() {
    leftMotor.set(Constants.Elevator.testSpeed);
  }
  public void testDown() {
    leftMotor.set(-Constants.Elevator.testSpeed);
  }

  @AutoLogOutput
  public double calculateOutput() {
    double P = ElevatorControl.calculate(getLocation(), targetHeight);
    P += Constants.Elevator.gravityFF;
    
    //if (this.MaxSpeedMode) P = Math.signum(P);
    if (P > .60) P = .60;
    if (P < -.25) P = -.25;
    return P;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (!this.movingToTarget) {
      stop();
      return;
    }


    
    leftMotor.set(calculateOutput());  

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

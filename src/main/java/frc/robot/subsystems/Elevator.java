// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public SparkMax leftMotor;
  public SparkMax rightMotor;  
  public SparkMaxConfig leftMotorConfig;
  public SparkMaxConfig rightMotorConfig;
  public Encoder elevatorEncoder;

  private double targetHeight = 0;
  private boolean movingToTarget = false;

  private double zeroOffset = 0;
  private double totalRevolutions = 0;
  private double lastPosition = 0;
  
  public enum Targets {
    Floor,
    Processor,
    Intake,
    L1,
    L2,
    L3,
    L4,
    NET
  }

  public Elevator() {
    leftMotor = new SparkMax(Constants.Elevator.leftMotor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    rightMotor = new SparkMax(Constants.Elevator.rightMotor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    leftMotorConfig = new SparkMaxConfig();
    leftMotorConfig.inverted(false);
    leftMotorConfig.idleMode(IdleMode.kBrake);

    rightMotorConfig = new SparkMaxConfig();
    rightMotorConfig.inverted(true);
    rightMotorConfig.follow(leftMotor, true);
    rightMotorConfig.idleMode(IdleMode.kBrake);

    rightMotor.configure(rightMotorConfig, null, null);
    leftMotor.configure(leftMotorConfig, null, null);

    elevatorEncoder = new Encoder(1, 2);

  }


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public boolean isAtLocation() {
    return (Math.abs((elevatorEncoder.get()) - targetHeight) < Constants.Elevator.heightTolerance );
  }

  public boolean GoToTarget(Elevator.Targets target) {
    double targetHeight = 0;
    switch(target) {
      case Floor:
        targetHeight = Constants.Elevator.SeaFloorHeight;
        break;
      case Processor:
        break;
      case L1:
        break;
      case L2:
        break;
      case L3:
        break;
      case L4:
        break;
      case NET:
        break;
      default:

    }

    return goToLocation(targetHeight);
  }

  public boolean goToLocation(double targetHeight) {
    this.targetHeight = targetHeight;
    if (isAtLocation()) return true;
    this.movingToTarget = true;
    return false;
  }

  public double getLocation() {
    return -elevatorEncoder.getDistance();
  }

  public boolean armClear() {
    return getLocation() >= Constants.Elevator.ArmClearHeight;
  }


  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void dunk() {
    this.targetHeight = this.targetHeight - Constants.Elevator.DunkDistance;
    this.movingToTarget = true;
  }

  public void testUp() {
    leftMotor.set(Constants.Elevator.testSpeed);
  }

  public void testDown() {
    leftMotor.set(-Constants.Elevator.testSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (this.movingToTarget) {
      if (!isAtLocation()) {
        leftMotor.set( (getLocation() < targetHeight) ? Constants.Elevator.elevatorSpeed : -Constants.Elevator.elevatorSpeed);
      } else {
        this.movingToTarget = false;
        stop();

      } 
    }

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

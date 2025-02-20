// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.Constants;
public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public SparkMax LeftMotor;
  public SparkMax RightMotor;
  public SparkMaxConfig LeftMotorConfig;
  public SparkMaxConfig RightMotorConfig;
  public DutyCycleEncoder armEncoder;

  private double targetAngle = 0;
  private boolean movingToTarget = false;

  private Elevator m_elevator;

  public Arm(Elevator elevator) {
    m_elevator = elevator;
    LeftMotor = new SparkMax(Constants.Arm.leftMotor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    RightMotor = new SparkMax(Constants.Arm.rightMotor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    LeftMotorConfig = new SparkMaxConfig();
    LeftMotorConfig.inverted(true);
    LeftMotorConfig.idleMode(IdleMode.kBrake);
    LeftMotorConfig.disableFollowerMode();
    LeftMotorConfig.openLoopRampRate(Constants.Arm.rampUpTime);

    LeftMotor.configure(LeftMotorConfig, null, null);

    
    RightMotorConfig = new SparkMaxConfig();
    RightMotorConfig.inverted(false);
    RightMotorConfig.idleMode(IdleMode.kBrake);
    RightMotorConfig.follow(LeftMotor, true);
    RightMotorConfig.openLoopRampRate(Constants.Arm.rampUpTime);

    RightMotor.configure(RightMotorConfig, null, null);

    armEncoder = new DutyCycleEncoder(Constants.Arm.Encoder);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public boolean isAtLocation() {
    return (Math.abs(armDegrees() - targetAngle) < Constants.Arm.angleTolerance );
  }

  public double armDegrees() {
    double raw = armEncoder.get() + Constants.Arm.encoderOffset;
    while (raw < 0) raw += 1;
    while (raw >= 1) raw -= 1; // max degrees will be 359.9-ish, 360 will be 0
    return raw * 360;
  }
  public boolean goToLocation(double target) {
    this.targetAngle = target;
    if (isAtLocation()) return true;
   this.movingToTarget = true;
    return false;
  }

  public void stop() {
    LeftMotor.stopMotor();
  }

  public void testUp() {
    LeftMotor.set(Constants.Arm.testSpeed);
  }

  public void testDown() {
    LeftMotor.set(-Constants.Arm.testSpeed);
  }

  public void hold() {
    LeftMotor.set(Constants.Arm.holdSpeed);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (this.movingToTarget) {
      if (!isAtLocation()) {

        if (m_elevator.armClear() || targetAngle == Constants.Arm.DownAngle) {
          LeftMotor.set( (armDegrees() < targetAngle) ? Constants.Arm.armSpeed : -Constants.Arm.armSpeed);
        } else {
          stop();
        }
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

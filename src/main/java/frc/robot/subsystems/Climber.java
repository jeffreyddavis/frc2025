// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public SparkMax LeftMotor;

  public SparkMax RightMotor;
  public SparkMaxConfig LeftMotorConfig;
  public SparkMaxConfig RightMotorConfig;
  // public DutyCycleEncoder ClimberEncoder;
  public Servo leftServo;
  public Servo rightServo;
  private double targetAngle = 0;
  private boolean movingToTarget = false;

  public Climber() {
    //LeftMotor =
    //    new SparkMax(
    //        Constants.Climber.leftMotor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    RightMotor =
        new SparkMax(
            Constants.Climber.rightMotor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    //LeftMotorConfig = new SparkMaxConfig();
    //LeftMotorConfig.inverted(true);
    //LeftMotorConfig.idleMode(IdleMode.kBrake);
    //LeftMotorConfig.disableFollowerMode();
    //LeftMotorConfig.openLoopRampRate(Constants.Climber.rampUpTime);

    //LeftMotor.configure(LeftMotorConfig, null, null);

    //leftServo = new Servo(1);
    rightServo = new Servo(0);

    RightMotorConfig = new SparkMaxConfig();
    RightMotorConfig.inverted(false);
    RightMotorConfig.idleMode(IdleMode.kBrake);
    RightMotorConfig.disableFollowerMode();
    RightMotorConfig.openLoopRampRate(Constants.Climber.rampUpTime);

    RightMotor.configure(RightMotorConfig, null, null);

    // ClimberEncoder = new DutyCycleEncoder(Constants.Climber.Encoder);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public boolean isAtLocation() {
    return (Math.abs(ClimberDegrees() - targetAngle) < Constants.Climber.angleTolerance);
  }

  public double ClimberDegrees() {
    // double raw = ClimberEncoder.get() + Constants.Climber.encoderOffset;
    double raw = 0;
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

  public void idle() {
    enableClimbing();
    stop();
  }

  public void stop() {
    RightMotor.stopMotor();
  }

  public void testUp() {
    RightMotor.set(Constants.Climber.testSpeed);
  }

  public void testDown() {
    RightMotor.set(-Constants.Climber.testSpeed);
  }

  public void releaseCage() {
    rightServo.setPosition(.7);
  }

  public void enableClimbing() {
    //leftServo.setPosition(1);
    rightServo.setPosition(0);
  }

  public void hold() {
    RightMotor.set(Constants.Climber.holdSpeed);
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
        RightMotor.set(
            (ClimberDegrees() < targetAngle) ? Constants.Climber.Speed : -Constants.Climber.Speed);
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

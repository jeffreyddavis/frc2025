// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public SparkFlex leftMotor;

  public SparkFlex rightMotor;
  public SparkFlexConfig leftMotorConfig;
  public SparkFlexConfig rightMotorConfig;
  public DutyCycleEncoder hopperEncoder;

  public Hopper() {
    leftMotor =
        new SparkFlex(
            Constants.Hopper.leftMotor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    rightMotor =
        new SparkFlex(
            Constants.Hopper.rightMotor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    leftMotorConfig = new SparkFlexConfig();
    leftMotorConfig.inverted(false);
    leftMotorConfig.idleMode(IdleMode.kCoast);

    leftMotor.configure(leftMotorConfig, null, null);

    rightMotorConfig = new SparkFlexConfig();
    rightMotorConfig.inverted(true);
    rightMotorConfig.idleMode(IdleMode.kCoast);
    rightMotorConfig.disableFollowerMode();
    rightMotor.configure(rightMotorConfig, null, null);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void stop() {
    setSpeed(0);
  }

  public void setSpeed(double Speed) {
    leftMotor.set(Speed);
    rightMotor.set(Speed);
  }

  public void testIn() {
    setSpeed(Constants.Hopper.testSpeed);
  }

  public void testOut() {
    setSpeed(-Constants.Hopper.testSpeed);
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

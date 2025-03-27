// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public SparkFlex Motor;

  public SparkFlexConfig MotorConfig;
  public DutyCycleEncoder intakeEncoder;

  private final DigitalInput m_IntakeLimitSwitch = new DigitalInput(Constants.Intake.intakeLimitSwitchId);
  private final DigitalInput m_IntakeLimitSwitch2 = new DigitalInput(Constants.Intake.intakeLimitSwitch2Id);

  public Intake() {
    Motor =
        new SparkFlex(
            Constants.Intake.Motor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    MotorConfig = new SparkFlexConfig();
    MotorConfig.inverted(false);
    MotorConfig.idleMode(IdleMode.kBrake);

    Motor.configure(MotorConfig, null, null);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void stop() {
    Motor.stopMotor();
  }

  @AutoLogOutput
  public boolean SeesCoral() {
    return !m_IntakeLimitSwitch.get() || !m_IntakeLimitSwitch2.get();
  }

  public void brake() {
    MotorConfig.idleMode(IdleMode.kBrake);
    Motor.configure(MotorConfig, null, null);
  }

  public void coast() {
    MotorConfig.idleMode(IdleMode.kCoast);
    Motor.configure(MotorConfig, null, null);
  }

  public void DoIntake() {
    Motor.set(Constants.Intake.intakeAlgaeSpeed);
  }

  public void ShootAlgae() {
    Motor.set(Constants.Intake.shootAlgaeSpeed);
  }

  public void testIn() {
    Motor.set(Constants.Intake.testSpeed);
  }

  public void testOut() {
    Motor.set(-Constants.Intake.testSpeed);
  }

  public void holdAlgae() {
    Motor.set(Constants.Intake.holdSpeed);
  }

  public void ShootCoral() {
    Motor.set(Constants.Intake.shootCoralSpeed);
  }
  public void ShootCoralAuto() {
    Motor.set(Constants.Intake.shootCoralSpeedAuto);
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

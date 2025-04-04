// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public SparkMax LeftMotor;

  // public SparkMax RightMotor;
  public SparkMaxConfig LeftMotorConfig;
  // public SparkMaxConfig RightMotorConfig;
  public Encoder ClimberEncoder;
  public Servo leftServo;
  //public Servo rightServo;
  @AutoLogOutput
  private double targetHeight = 0;
  @AutoLogOutput
  private boolean movingToTarget = false;

  public Climber() {

    SmartDashboard.putBoolean("Went to height", false);

    LeftMotor =
        new SparkMax(
            Constants.Climber.leftMotor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

//    RightMotor =
//        new SparkMax(
//            Constants.Climber.rightMotor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    LeftMotorConfig = new SparkMaxConfig();
    LeftMotorConfig.inverted(true);
    LeftMotorConfig.idleMode(IdleMode.kBrake);
    LeftMotorConfig.disableFollowerMode();
    //LeftMotorConfig.openLoopRampRate(Constants.Climber.rampUpTime);

    LeftMotor.configure(LeftMotorConfig, null, null);

    leftServo = new Servo(1);
   // rightServo = new Servo(0);

    //RightMotorConfig = new SparkMaxConfig();
    //RightMotorConfig.inverted(false);
    //RightMotorConfig.idleMode(IdleMode.kBrake);
    //RightMotorConfig.follow(LeftMotor, true);
    //RightMotorConfig.openLoopRampRate(Constants.Climber.rampUpTime);

    //RightMotor.configure(RightMotorConfig, null, null);

    ClimberEncoder = new Encoder(Constants.Climber.Encoder, Constants.Climber.otherencoder);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  @AutoLogOutput
  public boolean isAtHeight() {
    return (Math.abs(ClimberHeight() - targetHeight) < Constants.Climber.Tolerance);
  }
@AutoLogOutput
  public double ClimberHeight() {
    double raw = ClimberEncoder.get() + Constants.Climber.encoderOffset;
    return raw;
  }

  public void goToHeight(double target) {
    SmartDashboard.putBoolean("Went to height", true);
    this.targetHeight = target;
    SmartDashboard.putNumber("Target height", target);
    this.movingToTarget = true;
  }

  public void idle() {
    enableClimbing();
    stop();
  }

  public void stop() {
    LeftMotor.stopMotor();
    // RightMotor.stopMotor();
  }

  public void testUp() {
    LeftMotor.set(Constants.Climber.testSpeed);
  }

  public void testDown() {
    LeftMotor.set(-Constants.Climber.testSpeed);
  }

  public void releaseCage() {
    // rightServo.setPosition(.7);
    leftServo.setPosition(.15);
    
  }

  public void enableClimbing() {
    leftServo.setPosition(1);
    // rightServo.setPosition(0);
  }

  public void hold() {
    LeftMotor.set(Constants.Climber.holdSpeed);
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
      if (!isAtHeight()) {
       LeftMotor.set(
          (ClimberHeight() < targetHeight) ? Constants.Climber.Speed : -Constants.Climber.Speed);
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
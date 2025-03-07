// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.addons.PIDMint;
import edu.wpi.first.math.util.Units;

public class Arm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public SparkMax LeftMotor;

  //public SparkMax RightMotor;
  public SparkMaxConfig LeftMotorConfig;
  //public SparkMaxConfig RightMotorConfig;
  public DutyCycleEncoder armEncoder;

  @AutoLogOutput
  private double targetAngle = 0;
  @AutoLogOutput
  private boolean movingToTarget = false;

  private Elevator m_elevator;
  private PIDMint armControlMint;


  @AutoLogOutput
  public boolean MaxSpeedMode = false;

  public Arm(Elevator elevator) {
    m_elevator = elevator;
    LeftMotor =
        new SparkMax(
            Constants.Arm.leftMotor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    //RightMotor =
    //    new SparkMax(
    //        Constants.Arm.rightMotor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    LeftMotorConfig = new SparkMaxConfig();
    LeftMotorConfig.inverted(false);
    LeftMotorConfig.idleMode(IdleMode.kBrake);
    LeftMotorConfig.disableFollowerMode();
    LeftMotorConfig.openLoopRampRate(Constants.Arm.rampUpTime);

    LeftMotor.configure(LeftMotorConfig, null, null);

    //RightMotorConfig = new SparkMaxConfig();
    //RightMotorConfig.inverted(false);
    //RightMotorConfig.idleMode(IdleMode.kBrake);
    //RightMotorConfig.follow(LeftMotor, true);
    //RightMotorConfig.openLoopRampRate(Constants.Arm.rampUpTime);

    //RightMotor.configure(RightMotorConfig, null, null);

    armEncoder = new DutyCycleEncoder(Constants.Arm.Encoder);

    armControlMint = new PIDMint(.007, 0, 0, 0, 0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */

  @AutoLogOutput
  public String CurrentCommand() {
     return this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "None";
  }

  @AutoLogOutput
  public boolean isAtLocation() {

    boolean result = (Math.abs(armDegrees() - targetAngle) < Constants.Arm.angleTolerance);
    return (Math.abs(armDegrees() - targetAngle) < Constants.Arm.angleTolerance);
  }

  @AutoLogOutput
  public boolean isNearLocation() {
    return (Math.abs(armDegrees() - targetAngle) < (Constants.Arm.angleTolerance*3));
  }

  public void setMaxSpeedMode(boolean max) {
    this.MaxSpeedMode = max;
  }

  public void dunk() {
    this.targetAngle = armDegrees() - 30;
    this.movingToTarget = true;
  }

  @AutoLogOutput
  public double armDegrees() {
    double raw = armEncoder.get() + Constants.Arm.encoderOffset;
    while (raw < 0) raw += 1;
    while (raw >= 1) raw -= 1; // max degrees will be 359.9-ish, 360 will be 0
    return raw * 360;
  }

  public boolean goToLocation(double target) {
    this.targetAngle = target;
    this.movingToTarget = true;
    return isAtLocation();
    
  }

  public void stop() {
    LeftMotor.set(Constants.Arm.gravityFF * sineOfAngle());
    this.movingToTarget = false;
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
  @AutoLogOutput
  public double sineOfAngle(){
    return Math.sin(Units.degreesToRadians(armDegrees()));
  }
  @AutoLogOutput
  public double calculateOutput() {
    double output = armControlMint.calculate(armDegrees(), targetAngle);


    output += (Constants.Arm.gravityFF * sineOfAngle());

    if (this.MaxSpeedMode) output = Math.signum(output);
    return output;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    if (!this.movingToTarget) { 
      //stop();
      return;
    }


    //if (m_elevator.armClear() || targetAngle == Constants.Arm.DownAngle || targetAngle == Constants.Arm.startingPosition) {
      
      LeftMotor.set(calculateOutput());
    //}
        
      
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterSubsystemConstants;

public class BallShooter extends SubsystemBase {
  private static BallShooter instance;

  private CANSparkMax shooterMotor = new CANSparkMax(ShooterSubsystemConstants.kFeederMotorPort,
      MotorType.kBrushless); // TODO: Remove Snake casing
  private CANSparkMax feederMotor = new CANSparkMax(ShooterSubsystemConstants.kShooterMotorPort,
      MotorType.kBrushless);
  private RelativeEncoder shooterMotorEncoder = shooterMotor.getEncoder();
  private RelativeEncoder feederMotorEncoder = feederMotor.getEncoder();
  private SparkMaxPIDController shooterMotorPID = shooterMotor.getPIDController();
  private SparkMaxPIDController feederMotorPID = feederMotor.getPIDController();
  private double setpoint;

  public static BallShooter getInstance() {
    if (instance == null) {
      instance = new BallShooter();
    }
    return instance;
  }

  private BallShooter() {
    shooterMotor.restoreFactoryDefaults();
    feederMotor.restoreFactoryDefaults(); // TODO: Set inverted settings of the motors

    shooterMotor.setIdleMode(IdleMode.kCoast);
    feederMotor.setInverted(ShooterSubsystemConstants.kInvertedFeederMotor);
    shooterMotor.setInverted(ShooterSubsystemConstants.kInvertedShooterMotor);
    shooterMotorEncoder.setPositionConversionFactor(ShooterSubsystemConstants.driveConversionPositionFactor);
    shooterMotorEncoder.setVelocityConversionFactor(ShooterSubsystemConstants.driveConversionVelocityFactor);
    feederMotorEncoder.setPositionConversionFactor(ShooterSubsystemConstants.driveConversionPositionFactor);
    feederMotorEncoder.setVelocityConversionFactor(ShooterSubsystemConstants.driveConversionVelocityFactor);

    shooterMotor.burnFlash();
    feederMotor.burnFlash();

  }

  public void startShooterMotor(int motorPower) {
    shooterMotor.set(MathUtil.clamp(motorPower, ShooterSubsystemConstants.kMinMotorPower,
        ShooterSubsystemConstants.kMaxMotorPower)); // TODO: Make a constant for the motor power

  }

  public void startFeederMotor(int motorPower) {
    feederMotor.set(MathUtil.clamp(motorPower, ShooterSubsystemConstants.kMinMotorPower,
        ShooterSubsystemConstants.kMaxMotorPower));
  }

  public void stopShooterMotor() {
    shooterMotor.stopMotor();
  }

  public void stopFeederMotor() {
    feederMotor.stopMotor();
  }

  public double getShooterVelocity() {
    return shooterMotorEncoder.getVelocity();
  }

  public double getShooterPosition() {
    return shooterMotorEncoder.getPosition();
  }

  public double getFeederVelocity() {
    return feederMotorEncoder.getVelocity();
  }

  public double getFeederPosition() {
    return feederMotorEncoder.getPosition();
  }

  public void setSetpointShooterMotor(double setpoint) {
    this.setpoint = setpoint;
    shooterMotorEncoder.setPosition(setpoint);
  }

  public void setSetpointFeederMotor(double setpoint) {
    this.setpoint = setpoint;
    feederMotorEncoder.setPosition(setpoint);
  }

  

}

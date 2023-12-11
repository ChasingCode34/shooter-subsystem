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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallShooter extends SubsystemBase{
    private static BallShooter instance;

    private CANSparkMax shooterMotor = new CANSparkMax(Constants.ShooterSubsystemConstants.kMotor1Port, MotorType.kBrushless); //Improve names
    private CANSparkMax feederMotor = new CANSparkMax(Constants.ShooterSubsystemConstants.kMotor2Port, MotorType.kBrushless);
    private RelativeEncoder encoder_shooterMotor = shooterMotor.getEncoder();
    private RelativeEncoder encoder_feederMotor = feederMotor.getEncoder();
    private SparkMaxPIDController PID_shooterMotor = shooterMotor.getPIDController();
    private SparkMaxPIDController PID_feederMotor = feederMotor.getPIDController();
    private double setpoint;

    public static BallShooter getInstance() {
        if(instance == null) {
            instance = new BallShooter();
        }
        return instance;
    }

    private BallShooter() {
        shooterMotor.restoreFactoryDefaults();
        feederMotor.restoreFactoryDefaults(); //TODO: Set idle mode of the motors

        shooterMotor.setIdleMode(IdleMode.kCoast);
        feederMotor.setIdleMode(IdleMode.kCoast);
        shooterMotor.burnFlash();
        feederMotor.burnFlash();
        encoder_shooterMotor.setPositionConversionFactor(Constants.ShooterSubsystemConstants.driveConversionPositionFactor); //TODO: Set position conversion factor and velocity conversion factors for both motors
        encoder_shooterMotor.setVelocityConversionFactor(Constants.ShooterSubsystemConstants.driveConversionVelocityFactor);
        encoder_feederMotor.setPositionConversionFactor(Constants.ShooterSubsystemConstants.driveConversionPositionFactor);
        encoder_feederMotor.setVelocityConversionFactor(Constants.ShooterSubsystemConstants.driveConversionVelocityFactor);
        
    }

    //TODO: Adjust method names for the shoot and suck methods to be more precise
    public void startShooterMotor(int motorPower) {
        shooterMotor.set(motorPower); //TODO: Make a constant for the motor power
        
        
    }

    public void startFeedMotor(int motorPower) {
      feederMotor.set(motorPower);
    }

    public void stopShooterMotor() {
        shooterMotor.stopMotor();
    }

    public void stopFeederMotor() {
       feederMotor.stopMotor();
    }

    public void setSetpointShooterMotor(double setpoint) {
      this.setpoint = setpoint;
      encoder_shooterMotor.setPosition(setpoint);
    }

    public void setSetpointFeederMotor(double setpoint) {
      this.setpoint = setpoint;
      encoder_feederMotor.setPosition(setpoint);
    }


}

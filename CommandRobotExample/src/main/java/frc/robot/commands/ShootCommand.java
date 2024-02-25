// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.BallShooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ShootCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final BallShooter m_ballShooter = BallShooter.getInstance();
  Timer timer = new Timer();
  private int motorPower;
  private int secondsToRun;
  private int count = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootCommand(int motorPower, int secondsToRun) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_ballShooter);
    this.motorPower = motorPower;
    this.secondsToRun = secondsToRun;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.start();
    while(timer.get() < secondsToRun) {
      m_ballShooter.startFeederMotor(motorPower);
      count++;
    }
    m_ballShooter.stopFeederMotor();
    timer.reset();
    timer.start();
    while(timer.get() < secondsToRun) {
      m_ballShooter.startShooterMotor(motorPower);
      count++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ballShooter.stopShooterMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count == 2;
  }
}

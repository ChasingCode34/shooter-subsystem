package frc.robot.commands;

import frc.robot.subsystems.BallShooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FeederCommand extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final BallShooter m_ballShooter = BallShooter.getInstance();
    Timer timer = new Timer();
    private int motorPower;
    private int secondsToRun;

    public FeederCommand(int motorPower, int secondsToRun) {
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
        m_ballShooter.startFeedMotor(motorPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
        m_ballShooter.stopFeederMotor();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return secondsToRun >= timer.get() ;
  }
}

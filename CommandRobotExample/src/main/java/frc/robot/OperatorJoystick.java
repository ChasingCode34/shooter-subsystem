package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.BallShooter;

public class OperatorJoystick extends XboxController{

    public final JoystickButton xButton;
    public final JoystickButton bButton;
    private BallShooter shooter = BallShooter.getInstance();

    private static OperatorJoystick instance;

    public static OperatorJoystick getInstance() {
        if(instance == null) {
            instance = new OperatorJoystick(Constants.OperatorConstants.kOperatorJoystickPort);
        }
        return instance;
    }

    private OperatorJoystick(int port) {
        super(port);
        xButton = new JoystickButton(this, XboxController.Button.kX.value);
        bButton = new JoystickButton(this, XboxController.Button.kB.value);

        xButton.onTrue(new ShootCommand(1, 2));
        bButton.onTrue(new FeederCommand(1, 2));
        
    }

    
    
}

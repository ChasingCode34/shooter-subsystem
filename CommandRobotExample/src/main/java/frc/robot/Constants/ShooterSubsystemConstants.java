package frc.robot.Constants;

public class ShooterSubsystemConstants {
    public static final int kFeederMotorPort = -1;
    public static final int kShooterMotorPort = -1;

    public static final boolean kInvertedFeederMotor = false;
    public static final boolean kInvertedShooterMotor = false;

    public static final double driveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double wheelDiameter = 4.0;
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;

    public static final double kMinMotorPower = 0;
    public static final double kMaxMotorPower = 0.6;
}

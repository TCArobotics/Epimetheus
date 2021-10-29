package frc.robot;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.XboxController;

public class ManipulatorControl
{
    //motors
    public static SpeedController m_TurntableMotor;
    public static SpeedController m_ShooterMotor;

    public final XboxController XboxController;
    private final Debouncer AButton;

    //constants
    private final double turntableSpeed;
    private final double shooterSpeed;

    private boolean isShooting;

    public ManipulatorControl()
    {
        m_TurntableMotor = new PWMVictorSPX(RobotMap.kTurntablePort);
        m_ShooterMotor = new PWMVictorSPX(RobotMap.kShooterPort);

        XboxController = new XboxController(RobotMap.kDriverControllerPort);;
        AButton = new Debouncer(XboxController, RobotMap.kAPort);

        turntableSpeed = .5;
        shooterSpeed = .5;

        isShooting = false;
    }

    public void calculate()
    {
        isShooting = (AButton.get()) ? !isShooting : isShooting; //toggle between shooting
    }

    public void execute()
    {
        m_TurntableMotor.set(isShooting ? turntableSpeed : 0);
        m_ShooterMotor.set(isShooting ? shooterSpeed : 0);
    }
}
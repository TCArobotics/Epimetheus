package frc.robot;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.XboxController;

public class ManipulatorControl
{
    //motors
    public static SpeedController m_GroundIntakeMotor;
    public static SpeedController m_ShooterIntakeMotor;
    public static SpeedController m_TurntableMotor;
    public static SpeedController m_ShooterRightMotor;
    public static SpeedController m_ShooterLeftMotor;

    public final XboxController XboxController;
    private final Debouncer XButton;
    private final Debouncer YButton;

    //constants
    private final double groundIntakeSpeed;
    private final double shooterIntakeSpeed;
    private final double turntableSpeed;
    private final double shooterRightSpeed;
    private final double shooterLeftSpeed;

    private boolean isIntaking;
    private boolean isShooting;

    public ManipulatorControl(XboxController _XboxController)
    {
        

        m_GroundIntakeMotor = new PWMVictorSPX(RobotMap.kGroundIntakePort);
        m_ShooterIntakeMotor = new PWMVictorSPX(RobotMap.kShooterIntakePort);
        m_TurntableMotor = new PWMVictorSPX(RobotMap.kTurntablePort);
        m_ShooterRightMotor = new PWMVictorSPX(RobotMap.kShooterRightPort);
        m_ShooterLeftMotor = new PWMVictorSPX(RobotMap.kShooterLeftPort);

        this.XboxController = _XboxController;
        XButton = new Debouncer(XboxController, RobotMap.kYPort);
        YButton = new Debouncer(XboxController, RobotMap.kXPort);

        groundIntakeSpeed = .5;
        shooterIntakeSpeed = .5;
        turntableSpeed = .5;
        shooterRightSpeed = -.5;  //right and left should be opposite each other
        shooterLeftSpeed = .5;

        isIntaking = false;
        isShooting = false;
    }

    public void calculate()
    {
        isIntaking = (XButton.get()) ? !isIntaking : isIntaking; //toggle between intaking
        isShooting = (YButton.get()) ? !isShooting : isShooting; //toggle between shooting
    }

    public void execute()
    {
        m_GroundIntakeMotor.set(isIntaking ? groundIntakeSpeed : 0); //turns on ground intake motor

        m_ShooterIntakeMotor.set(isShooting ? shooterIntakeSpeed : 0); //turns on turntable, shooter intake, and shooter motors
        m_TurntableMotor.set(isShooting ? turntableSpeed : 0);
        m_ShooterRightMotor.set(isShooting ? shooterRightSpeed : 0);
        m_ShooterLeftMotor.set(isShooting ? shooterLeftSpeed : 0);
    }
}
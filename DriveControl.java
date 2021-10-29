    package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.Timer;


public class DriveControl
{
    //Controller and buttons
    private final XboxController XboxController;
    private final Debouncer startButton;
    private final Debouncer lbButton;
    private final Debouncer rbButton;
    private final Debouncer aButton;

    //Controllers for motors
    public static SpeedController m_frontLeft;
    public static SpeedController m_rearLeft; 
    public static SpeedController m_frontRight;
    public static SpeedController m_rearRight;

    //Controllers for manipulators
    public static PWMVictorSPX shooter;

    //Combines controllers into differential drive
    private final SpeedControllerGroup m_left;
    private final SpeedControllerGroup m_right;
    private final DifferentialDrive m_robotDrive;

    //Constants
    private final double kdistanceBtwnWheels = 1.81; //Distance between wheels in ft
    private final double kleftMultiplier = 1; //Value to multiply left speed by
    private final double krightMultiplier = 1; //Value to multiply right speed by
    private final Double[] barrelTimings = {5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0};    //stores how long each step of autonomous takes. (fine tune)
    private final Double[] slalomTimings = {.4, 1.5, 2.5, 4.0, 5.2, 7.6, 8.4, 14.5, 16.5, 17.4, 19.2, 22.2, 24.2, 100.0};
    private final Double[] bounceTimings = {5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0};

    //Variables that do not have default values
    private double LeftDriveInput; //Stores the actual left input value for execute
    private double RightDriveInput; //Stores the actual right input value for execute
    private int selectedDrive; //Stores the actual drive type used by the execute function (1 = arcade, 2 = curvature, 3 = tank)
    private boolean driveType; //Stores the state of drive for joysticks (1 = arcade/curvature, 2 = tank)
    private double speed; //Stores the speed the robot is going (-0.5 or -1)
    private double shooterSpeed; //Stores the speed of the shooter (either 0 or 1)
    private boolean isStopped; //Stores if the robot is stopped
    private double m_autoTimerCurrent; //How much time has passed since program start
    public double m_autoTimerFirst;

    //initialize variables
    public DriveControl()
    {
        m_frontLeft = new PWMVictorSPX(RobotMap.kFrontLeftPort);
        m_rearLeft = new PWMVictorSPX(RobotMap.kRearLeftPort);
        m_frontRight = new PWMVictorSPX(RobotMap.kFrontRightPort);
        m_rearRight = new PWMVictorSPX(RobotMap.kRearRightPort);

        shooter = new PWMVictorSPX(RobotMap.kShooterPort);

        m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
        m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
        m_robotDrive = new DifferentialDrive(m_left, m_right);
        
        XboxController = new XboxController(RobotMap.kDriverControllerPort);
        lbButton = new Debouncer(XboxController, RobotMap.kLBPort);
        rbButton = new Debouncer(XboxController, RobotMap.kRBPort);
        startButton = new Debouncer(XboxController, RobotMap.kStartPort);
        aButton = new Debouncer(XboxController, RobotMap.kAPort);

        driveType = true;
        selectedDrive = 0;
        speed = -.6;
        isStopped = false;
    }

    //gets controller inputs and stores them in various drive variables
    public void calculate()
    {
        //driveType = (rbButton.get()) ? !driveType : driveType;

        isStopped = (startButton.get()) ? !isStopped : isStopped;
        driveType = (rbButton.get()) ? !driveType : driveType;

        if(lbButton.get())
        {
            speed = (speed == -0.85) ? -0.6 : speed - 0.25;    //if speed goes past the max, reset the cycle, then subtract .25 from the speed
        }

        if(aButton.get())
        {
            shooterSpeed = (shooterSpeed == 1) ? 0 : 1;
        }

        LeftDriveInput = XboxController.getY(Hand.kLeft); //Default LeftDriveInput

        if (driveType) //If driveType is set to Arcade/Curvature
        {
            RightDriveInput = XboxController.getX(Hand.kRight);
            selectedDrive = (Math.abs(XboxController.getY(Hand.kLeft)) > 0.1) ? 0 : 1; //change selected drive if left bumper is pressed enough
        }

        else //If driveType is set to Tank
        {
            RightDriveInput = XboxController.getY(Hand.kRight);
            selectedDrive = 2; //change to tank drive
        }
    }

    //Calculates motor speeds for the robot to drive in a straight line
    public void calculateAutonomousDrive(double _currentTime, double _startTime, double _duration, double _rightInput, double _leftInput, double _speed)
    {
    	//Calculates whether the time span is correct for the function to run
    	if ((_currentTime >= _startTime) && (_currentTime <= _startTime + _duration))
        {
            this.selectedDrive = 2;
            this.RightDriveInput = _rightInput;
            this.LeftDriveInput = _leftInput;
            this.speed = _speed;
        }
    }  

    //Calculates motor speeds for the robot to drive in a circle (radius = inner wheel to point of rotation)
    public void calculateAutonomousCircle(double _currentTime, double _startTime, double _duration, double _speed, double _radius, boolean _isClockwise)
    {
        //Calculates whether the time span is correct for the function to run
        if ((_currentTime >= _startTime) && (_currentTime <= _startTime + _duration))
            {
                double circleCalc = _radius / (_radius + kdistanceBtwnWheels);
                this.selectedDrive = 2;
                this.speed = _speed;
                this.LeftDriveInput = (_isClockwise) ? 1 : circleCalc;
                this.RightDriveInput = (_isClockwise) ? circleCalc : 1;
            }
    }
    
    //Make the robot drive based on given inputs
    public void execute()
    {
        double leftSpeedCalc = (isStopped ? 0 : 1) * speed *  LeftDriveInput;
        double rightSpeedCalc = (isStopped ? 0 : 1) * speed * RightDriveInput;
        shooter.set(shooterSpeed);
        switch(selectedDrive)
        {
        case 0:
            m_robotDrive.curvatureDrive(leftSpeedCalc, -rightSpeedCalc*1.5, false);
            break;
        case 1:
            m_robotDrive.arcadeDrive(leftSpeedCalc, -rightSpeedCalc*.8);
            break;
        case 2:
            System.out.println("leftSpeedCalc: " + leftSpeedCalc);
            System.out.println("kleftMultiplier: " + kleftMultiplier);
            System.out.println("rightSpeedCalc: " + rightSpeedCalc);
            System.out.println("krightMultiplier: " + krightMultiplier);
            m_robotDrive.tankDrive(leftSpeedCalc * kleftMultiplier, rightSpeedCalc * krightMultiplier);
            break;
        }
    }

    //autonomous path code
    public void barrelRacing()
    {
        m_autoTimerCurrent = Timer.getFPGATimestamp();
        //forward for 7.5 ft
        calculateAutonomousDrive(m_autoTimerCurrent, 0, barrelTimings[0], 1.0, 1.0, -0.5);

        //drive in a circle with (radius (3ft), speed (-0.5))
        calculateAutonomousCircle(m_autoTimerCurrent, barrelTimings[0], barrelTimings[1], -0.5, 3, true);
    
        //forward for 9 ft
        calculateAutonomousDrive(m_autoTimerCurrent, barrelTimings[1], barrelTimings[2], 1.0, 1.0, -0.5);
    
        //drive in a circle with (radius (3ft), speed (-0.5))
        calculateAutonomousCircle(m_autoTimerCurrent, barrelTimings[2], barrelTimings[3], -0.6, 3, false);
    
        //forward for 7 ft
        calculateAutonomousDrive(m_autoTimerCurrent, barrelTimings[3], barrelTimings[4], 1.0, 1.0, -0.5);
    
        //drive in a circle with (radius (3ft), speed (-0.5))
        calculateAutonomousCircle(m_autoTimerCurrent, barrelTimings[4], barrelTimings[5], -0.6, 2, false);
    
        //forward for 20 ft
        calculateAutonomousDrive(m_autoTimerCurrent, barrelTimings[5], barrelTimings[6], 1.0, 1.0, -0.6);
    
        //execute function
        execute();
    }

    public void slalomPath()
    {
        m_autoTimerCurrent = Timer.getFPGATimestamp() - m_autoTimerFirst;

        //
        calculateAutonomousDrive(m_autoTimerCurrent, 0, slalomTimings[1], 1, 1, .6);

        //drive in a circle with (radius (0ft), speed (-0.5))
        calculateAutonomousCircle(m_autoTimerCurrent, slalomTimings[0], slalomTimings[1], .6, 1.7, false);

        //forward for 7 ft
        calculateAutonomousDrive(m_autoTimerCurrent, slalomTimings[1], slalomTimings[2], 1, 1, .6);
    
        //drive in a circle with (radius (5ft), speed (-0.5))
        calculateAutonomousCircle(m_autoTimerCurrent, slalomTimings[2], slalomTimings[3], .6, 4.5, true);
    
        //forward for 7 ft
        calculateAutonomousDrive(m_autoTimerCurrent, slalomTimings[3], slalomTimings[4], 1, 1, .6);
    
        //drive in a circle with (radius (2.5ft), speed (-0.5))
        calculateAutonomousCircle(m_autoTimerCurrent, slalomTimings[4], slalomTimings[5], .6, 4, true);
    
        //forward for 7 ft
        calculateAutonomousDrive(m_autoTimerCurrent, slalomTimings[5], slalomTimings[6], 1, 1, 0.6);
    
        //drive in a circle with (radius (5ft), speed (-0.5))
        calculateAutonomousCircle(m_autoTimerCurrent, slalomTimings[6], slalomTimings[7], .6, 2.5, false);
    
        //forward for 7 ft
        calculateAutonomousDrive(m_autoTimerCurrent, slalomTimings[7], slalomTimings[8], 1, 1, 0.6);
    
        //drive in a circle with (radius (0ft), speed (-0.5))
        calculateAutonomousCircle(m_autoTimerCurrent, slalomTimings[8], slalomTimings[9], .6, 2, true);

        calculateAutonomousDrive(m_autoTimerCurrent, slalomTimings[9], slalomTimings[10], 1, 1, .6);

        calculateAutonomousCircle(m_autoTimerCurrent, slalomTimings[10], slalomTimings[11], .6, 3.5, true);

        calculateAutonomousCircle(m_autoTimerCurrent, slalomTimings[11], slalomTimings[12], .6, 3.5, false);
        
        calculateAutonomousDrive(m_autoTimerCurrent, slalomTimings[12], slalomTimings[13], 0, 0, 0.0);
    }

    public void bouncePath()
    {
        m_autoTimerCurrent = Timer.getFPGATimestamp() - m_autoTimerFirst;
        //Turn Left to face cone
        calculateAutonomousCircle(m_autoTimerCurrent, 0, bounceTimings[0], -0.5, 3, false);

        //Drive into cone A3
        calculateAutonomousDrive(m_autoTimerCurrent, bounceTimings[0], bounceTimings[1], 1.0, 1.0, -0.5);
    
        //Turn Left to face back away from cone while heading backwards
        calculateAutonomousCircle(m_autoTimerCurrent, bounceTimings[1], bounceTimings[2], -0.5, 5, false);
    
        //Drive Forward
        calculateAutonomousDrive(m_autoTimerCurrent, bounceTimings[2], bounceTimings[3], -1.0, -1.0, -0.5);
    
        //Circle Around D5 CCW
        calculateAutonomousCircle(m_autoTimerCurrent, bounceTimings[3], bounceTimings[4], 0.5, 5, false);
    
        //Head into cone A6
        calculateAutonomousDrive(m_autoTimerCurrent, bounceTimings[4], bounceTimings[5], -1.0, -1.0, -0.5);
    
        //Turn Left to face back away from cone while heading backwards
        calculateAutonomousCircle(m_autoTimerCurrent, bounceTimings[5], bounceTimings[6], -0.5, 5, false);
    
        //Drive Forward
        calculateAutonomousDrive(m_autoTimerCurrent, bounceTimings[6], bounceTimings[7], 1.0, 1.0, -0.5);
    
        //Turn around D7 CCW
        calculateAutonomousCircle(m_autoTimerCurrent, bounceTimings[7], bounceTimings[8], 0.5, 5, false);
    
        //Head forwards past D8
        calculateAutonomousDrive(m_autoTimerCurrent, bounceTimings[8], bounceTimings[9], 1.0, 1.0, -0.5);
    
        //Turn around D8 CCW
        calculateAutonomousCircle(m_autoTimerCurrent, bounceTimings[9], bounceTimings[10], 0.5, 5, false);
    
        //Head into cone A9
        calculateAutonomousDrive(m_autoTimerCurrent, bounceTimings[10], bounceTimings[11], 1.0, 1.0, -0.5);
    
        //Circle into finish zone
        calculateAutonomousCircle(m_autoTimerCurrent, bounceTimings[11], bounceTimings[12], -0.5, 15, false);    
    }
}
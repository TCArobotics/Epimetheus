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

    //Controllers for motors
    public static SpeedController m_frontLeft;
    public static SpeedController m_rearLeft; 
    public static SpeedController m_frontRight;
    public static SpeedController m_rearRight;

    //Combines controllers into differential drive
    private final SpeedControllerGroup m_left;
    private final SpeedControllerGroup m_right;
    private final DifferentialDrive m_robotDrive;

    //Constants
    private final double kdistanceBtwnWheels = .557; //Distance between wheels in meters
    private final double kleftMultiplier = 1; //Value to multiply left speed by
    private final double krightMultiplier = 1; //Value to multiply right speed by
    private final Double[] barrelTimings = {5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0};    //stores how long each step of autonomous takes. (fine tune)
    private final Double[] slalomTimings = {5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0};
    private final Double[] bounceTimings = {5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0};

    //Variables that do not have default values
    private double LeftDriveInput; //Stores the actual left input value for execute
    private double RightDriveInput; //Stores the actual right input value for execute
    private int selectedDrive; //Stores the actual drive type used by the execute function (1 = arcade, 2 = curvature, 3 = tank)
    private boolean driveType; //Stores the state of drive for joysticks (1 = arcade/curvature, 2 = tank)
    private double speed; //Stores the speed the robot is going (-0.5 or -1)
    private boolean isStopped; //Stores if the robot is stopped
    private double m_autoTimerCurrent; //How much time has passed since program start

    //initialize variables
    public DriveControl(XboxController _XboxController)
    {
        m_rearLeft = new PWMVictorSPX(RobotMap.kRearLeftPort);
        m_frontRight = new PWMVictorSPX(RobotMap.kFrontRightPort);
        m_rearRight = new PWMVictorSPX(RobotMap.kRearRightPort);

        m_left = new SpeedControllerGroup(m_frontLeft, m_rearLeft);
        m_right = new SpeedControllerGroup(m_frontRight, m_rearRight);
        m_robotDrive = new DifferentialDrive(m_left, m_right);
        
        this.XboxController = _XboxController;
        lbButton = new Debouncer(XboxController, RobotMap.kLBPort);
        rbButton = new Debouncer(XboxController, RobotMap.kRBPort);
        startButton = new Debouncer(XboxController, RobotMap.kStartPort);

        driveType = true;
        speed = -1;
        isStopped = false;
    }

    //gets controller inputs and stores them in various drive variables
    public void calculate()
    {
        driveType = (rbButton.get()) ? !driveType : driveType;

        isStopped = (startButton.get()) ? !isStopped : isStopped;

        if(lbButton.get())
        {
            speed = (speed == -1.25) ? -0.5 : speed -0.25;    //if speed goes past the max, reset the cycle, then subtract .25 from the speed
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
    public void calculateAutonomousDrive(double _currentTime, double _startTime, double _endTime, double _rightInput, double _leftInput, double _speed)
    {
    	//Calculates whether the time span is correct for the function to run
    	if ((_currentTime >= _startTime) && (_currentTime <= _endTime))
        {
            this.selectedDrive = 2;
            this.RightDriveInput = _rightInput;
            this.LeftDriveInput = _leftInput;
            this.speed = _speed;
        }
    }  

    //Calculates motor speeds for the robot to drive in a circle (radius = inner wheel to point of rotation)
    public void calculateAutonomousCircle(double _currentTime, double _startTime, double _endTime, double _speed, double _radius, boolean _isClockwise)
    {
        //Calculates whether the time span is correct for the function to run
        if ((_currentTime >= _startTime) && (_currentTime <= _endTime))
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
        switch(selectedDrive)
        {
        case 0:
            m_robotDrive.curvatureDrive(leftSpeedCalc, rightSpeedCalc, false);
            break;
        case 1:
            m_robotDrive.arcadeDrive(leftSpeedCalc, rightSpeedCalc);
            break;
        case 2:
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
        calculateAutonomousCircle(m_autoTimerCurrent, barrelTimings[2], barrelTimings[3], -0.5, 3, false);
    
        //forward for 7 ft
        calculateAutonomousDrive(m_autoTimerCurrent, barrelTimings[3], barrelTimings[4], 1.0, 1.0, -0.5);
    
        //drive in a circle with (radius (3ft), speed (-0.5))
        calculateAutonomousCircle(m_autoTimerCurrent, barrelTimings[4], barrelTimings[5], -0.5, 3, false);
    
        //forward for 20 ft
        calculateAutonomousDrive(m_autoTimerCurrent, barrelTimings[5], barrelTimings[6], 1.0, 1.0, -0.5);
    
        //execute function
        execute();
    }

    public void slalomPath()
    {
        m_autoTimerCurrent = Timer.getFPGATimestamp();
        //drive in a circle with (radius (0ft), speed (-0.5))
        calculateAutonomousCircle(m_autoTimerCurrent, 0, slalomTimings[0], -0.5, 0, false);

        //forward for 7 ft
        calculateAutonomousDrive(m_autoTimerCurrent, slalomTimings[0], slalomTimings[1], 1.0, 1.0, -0.5);
    
        //drive in a circle with (radius (5ft), speed (-0.5))
        calculateAutonomousCircle(m_autoTimerCurrent, slalomTimings[1], slalomTimings[2], -0.5, 5, true);
    
        //forward for 7 ft
        calculateAutonomousDrive(m_autoTimerCurrent, slalomTimings[2], slalomTimings[3], 1.0, 1.0, -0.5);
    
        //drive in a circle with (radius (2.5ft), speed (-0.5))
        calculateAutonomousCircle(m_autoTimerCurrent, slalomTimings[3], slalomTimings[4], -0.5, 2.5, false);
    
        //forward for 7 ft
        calculateAutonomousDrive(m_autoTimerCurrent, slalomTimings[4], slalomTimings[5], 1.0, 1.0, -0.5);
    
        //drive in a circle with (radius (5ft), speed (-0.5))
        calculateAutonomousCircle(m_autoTimerCurrent, slalomTimings[5], slalomTimings[6], -0.5, 5, true);
    
        //forward for 7 ft
        calculateAutonomousDrive(m_autoTimerCurrent, slalomTimings[6], slalomTimings[7], 1.0, 1.0, -0.5);
    
        //drive in a circle with (radius (0ft), speed (-0.5))
        calculateAutonomousCircle(m_autoTimerCurrent, slalomTimings[7], slalomTimings[8], -0.5, 5, false);
    }

    public void bouncePath()
    {
        m_autoTimerCurrent = Timer.getFPGATimestamp();
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
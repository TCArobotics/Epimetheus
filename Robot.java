/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/*  The VM is configured to automatically run this class, and to call the
    functions corresponding to each mode, as described in the TimedRobot
    documentation. If you change the name of this class or the package after
    creating this project, you must also update the build.gradle file in the
    project.  
*/
public class Robot extends TimedRobot 
{
  /*  Variable naming conventions: 
      camelCase, 
      k[name] for constants
  */
  
  final XboxController m_driverController = new XboxController(RobotMap.kDriverControllerPort);
  final DriveControl driveController = new DriveControl();

  private final String kbarrelRacing = "BarrelRacing"; //the different choices for autonomousChoice
  private final String kslalomPath = "SlalomPath";
  private final String kbouncePath = "BouncePath";
  private String autonomousChoice = kslalomPath; //which path to default to


  //This function is run when the robot is first started up.
  @Override
  public void robotInit() 
  {
  }

  //This function is called every robot packet, no matter the mode.

  @Override
  public void robotPeriodic() 
  {
  }

  @Override
  public void autonomousInit() 
  {
  }

  //This function is called periodically during autonomous.
  @Override
  public void autonomousPeriodic() 
  {
    switch(autonomousChoice) 
    {
    case kbarrelRacing:
      driveController.barrelRacing();
      break;
    case kslalomPath:
      driveController.slalomPath();
      break;
    case kbouncePath:
      driveController.bouncePath();
      break;
    }
  }

  //This function is called periodically during operator control.
  @Override
  public void teleopPeriodic() 
  {
    driveController.calculate();
    driveController.execute();
  }

  //This function is called periodically during test mode.
  @Override
  public void testPeriodic() 
  {
  }
}
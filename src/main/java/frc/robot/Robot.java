// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
// import com.pathplanner.lib.util.DriveFeedforwards;
// import com.pathplanner.lib.config.PIDConstants;
// import edu.wpi.first.wpilibj2.command.Command;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.auto.AutoBuilder;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  //private static final String kDefaultAuto = "Default";
  //private static final String kCustomAuto = "My Auto";

  // private static final Command RightNeutral = AutoBuilder.buildAuto("RightNeutralAuton");
  // private static final Command LeftNeutral = AutoBuilder.buildAuto("LeftNeutralAuton");
  // private static final Command OnlyShoot = AutoBuilder.buildAuto("OnlyShootBalls");
  // private static final Command HumanDump = AutoBuilder.buildAuto("HumanDump");

  // private Command m_autoSelected;
  // private final SendableChooser<Command> m_chooser;
  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code. 
   */
  public Robot() {
    // m_chooser = AutoBuilder.buildAutoChooser();
    // //m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.setDefaultOption("Do Nothing", new InstantCommand());
    // m_chooser.addOption("RightNeutralAuton", RightNeutral);
    // m_chooser.addOption("LeftNeutralAuton", LeftNeutral);
    // m_chooser.addOption("OnlyShootBalls", OnlyShoot);
    // m_chooser.addOption("HumanDump", HumanDump);
    // SmartDashboard.putData("Auto choices", m_chooser);
  }

  @Override
  public void robotInit() 
  {
    robotContainer = new RobotContainer();
  }
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    robotContainer.m_autoSelected = robotContainer.m_chooser.getSelected();
    if (robotContainer.m_autoSelected != null){
      CommandScheduler.getInstance().schedule(robotContainer.m_autoSelected);
    }
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + robotContainer.m_autoSelected);
  }

  /** This function is called periodically during autonomous. */

  // make sure to start spinning the hopper neo 
  @Override
  public void autonomousPeriodic() {
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     // Put custom auto code here
    //     break;
    //   case kDefaultAuto:
    //   default:
    //     // Put default auto code here
    //     break;
    // }
  }

  /** This function is called once when teleop is enabled. */
  // @Override
  // public void teleopInit() {
  //   if (autonomousCommand != null) {
  //     autonomousCommand.cancel();
  //   }
  // }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

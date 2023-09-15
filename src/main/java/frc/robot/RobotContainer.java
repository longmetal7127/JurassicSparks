// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import java.util.HashMap;
import java.util.List;

import javax.print.attribute.standard.MediaSize.NA;

import com.pathplanner.lib.*;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;

import frc.robot.commands.JoystickDrive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static HashMap<String, Command> eventMap = new HashMap<>();
  private final NavX navx = new NavX();
  private final DriveTrain drive = new DriveTrain(navx);

  public static Joystick joystick = new Joystick(Constants.OperatorConstants.kDriverJoystickPort);

  private static XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  private final JoystickDrive joystickDrive = new JoystickDrive(drive);

  private SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        drive::getPose, // Pose2d supplier
        drive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        drive::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        drive // The drive subsystem. Used to properly set the requirements of path following commands
    );
    
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drive.setDefaultCommand(joystickDrive);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    //new Trigger(m_exampleSubsystem::exampleCondition)
    //    .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    PathConstraints pc = new PathConstraints(3, 4);
    List<PathPlannerTrajectory> pathGroup = null;

    pathGroup = PathPlanner.loadPathGroup("auto1", pc);
    return autoBuilder.fullAuto(pathGroup);
 
    // An example command will be run in autonomous
   // return Autos.exampleAuto(m_exampleSubsystem);
  }
}

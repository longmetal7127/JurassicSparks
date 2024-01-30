// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.auto.AutoBuilder.TriFunction;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NavX;
import monologue.Logged;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Logged {

  // The robot's subsystems and commands are defined here...
  private final NavX navx = new NavX();
  private DriveTrain drive;

  public static CommandJoystick joystick = new CommandJoystick(
    Constants.OperatorConstants.kDriverJoystickPort
  );

  private static XboxController m_driverController = new XboxController(
    OperatorConstants.kDriverControllerPort
  );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private SendableChooser autoChooser;

  public RobotContainer()  {
    drive = new DriveTrain(navx);
    // Configure the trigger bindings
    configureBindings();
    drive.setDefaultCommand(
      new RunCommand(
        () -> {
          double multiplier = (((joystick.getThrottle() * -1) + 1) / 2); // turbo mode
          double z = RobotContainer.joystick.getZ() * -.8;

          drive.drive(
            MathUtil.applyDeadband(
              joystick.getY() * -multiplier,
              OperatorConstants.kDriveDeadband
            ),
            MathUtil.applyDeadband(
              joystick.getX() * -multiplier,
              OperatorConstants.kDriveDeadband
            ),
            MathUtil.applyDeadband(
              z ,
              OperatorConstants.kDriveDeadband
            ),
            true,
            true
          );
        },
        drive
      )
    );
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    NamedCommands.registerCommand("wheelsX", Commands.run(() -> drive.setX()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    JoystickButton wheelsX = new JoystickButton(
      m_driverController,
      XboxController.Button.kY.value
    );
    Trigger restartRoborio = joystick.trigger();
    restartRoborio.onTrue(new InstantCommand(()-> {
      navx.ahrs.zeroYaw();

    }));

    wheelsX.onTrue(Commands.run(() -> drive.setX()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
  }
}

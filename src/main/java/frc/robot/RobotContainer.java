// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.dense.block.MatrixOps_DDRB;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SpeakerAim;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.IntakeTrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.ShooterTrain;
import monologue.Logged;
import frc.robot.subsystems.ArmTrain;
import frc.robot.subsystems.ClimbTrain;

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
      Constants.OperatorConstants.kDriverJoystickPort);

  private static CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final ArmTrain armTrain = new ArmTrain();
  private final IntakeTrain intakeTrain = new IntakeTrain();
  private final ShooterTrain shooterTrain = new ShooterTrain();
  private final ClimbTrain climbTrain = new ClimbTrain();
  private Command shootCommand = new SequentialCommandGroup(
      intakeTrain.setSpeed(0.2),
      new WaitCommand(0.1),
      intakeTrain.setSpeed(0),
      shooterTrain.setSpeed(-2000),
      new WaitCommand(1),
      intakeTrain.setSpeed(-0.7),
      new WaitCommand(0.3),
      intakeTrain.setSpeed(0),
      shooterTrain.setSpeed(0)

  );
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {

    NamedCommands.registerCommand("shootPos", armTrain.setPosition(175 + 11 + 8));
    NamedCommands.registerCommand("sndShootPos", armTrain.setPosition(175 + 11 + 23));

        NamedCommands.registerCommand("trdShootPos", armTrain.setPosition(175 + 11 + 24));


    NamedCommands.registerCommand("shoot", shootCommand);
    NamedCommands.registerCommand("intake",
        new SequentialCommandGroup(armTrain.setPosition(175 + .5), intakeTrain.setSpeed(-5000)));
    NamedCommands.registerCommand("intakeOff",
        intakeTrain.setSpeed(0));

    drive = new DriveTrain(navx);

    configureBindings();
    drive.setDefaultCommand(
        new RunCommand(
            () -> {
              double multiplier = (((joystick.getThrottle() * -1) + 1) / 2); // turbo mode
              double z = RobotContainer.joystick.getZ() * -.9;

              drive.drive(
                  MathUtil.applyDeadband(
                      joystick.getY() * -multiplier,
                      OperatorConstants.kDriveDeadband),
                  MathUtil.applyDeadband(
                      joystick.getX() * -multiplier,
                      OperatorConstants.kDriveDeadband),
                  MathUtil.applyDeadband(
                      z,
                      OperatorConstants.kDriveDeadband),
                  true,
                  true);
            },
            drive));
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
  private Command getSpekAim() {
    return new SpeakerAim(drive, armTrain, false);
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    Trigger restartRoborio = joystick.trigger();
    restartRoborio.onTrue(new InstantCommand(() -> {
      navx.ahrs.zeroYaw();

    }));

    Trigger moveArm = m_driverController.a();
    moveArm.onTrue(armTrain.incrementPosition());
    Trigger moveArmD = m_driverController.b();
    moveArmD.onTrue(armTrain.decrementPosition());

    // Sysid
    /*
     * Trigger a = m_driverController.a();
     * a.onTrue(intakeTrain.quasistaticForward());
     * Trigger b = m_driverController.b();
     * b.onTrue(intakeTrain.quasistaticBackward());
     * Trigger x = m_driverController.x();
     * x.onTrue(intakeTrain.dynamicForward());
     * Trigger y = m_driverController.y();
     * y.onTrue(intakeTrain.dynamicBackward());
     */
    Trigger X = m_driverController.x();
    X.whileTrue(intakeTrain.setSpeed(-5000));
    X.onFalse(intakeTrain.setSpeed(0));

    Trigger Y = m_driverController.y();
    Y.onTrue(shootCommand);
    // Y.whileTrue(shooterTrain.setSpeed(-0.6));
    // Y.onFalse(shooterTrain.setSpeed(0));
    Trigger dpad = m_driverController.povLeft();
    dpad.whileTrue(shooterTrain.setSpeed(-200));
    dpad.onFalse(shooterTrain.setSpeed(0));

    Trigger r = m_driverController.povRight();
    r.onTrue(armTrain.setPosition(175 + 11 + 6.5));

    Trigger l = m_driverController.povDown();
    l.whileTrue(new SequentialCommandGroup(armTrain.setPosition(175 + .5), intakeTrain.setSpeed(-5000)));
    l.onFalse(intakeTrain.setSpeed(0));

    Trigger d = m_driverController.povUp();
    d.onTrue(armTrain.setPosition(273));
    Trigger bump = m_driverController.leftBumper();
    bump.onTrue(getSpekAim());

    Trigger rBump = m_driverController.rightBumper();
    rBump.onTrue(climbTrain.incrementPosition());

    Trigger rTrigger = m_driverController.rightTrigger();
    rTrigger.onTrue(climbTrain.decrementPosition());

        Trigger lTrigger = m_driverController.leftTrigger();
    lTrigger.onTrue(climbTrain.setPosition(515));

    // wheelsX.onTrue(Commands.run(() -> drive.setX()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
  }
}

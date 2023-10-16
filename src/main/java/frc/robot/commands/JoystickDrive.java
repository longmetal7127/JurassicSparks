package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
// who did the comments in this file they're actually vile
// 
public class JoystickDrive extends CommandBase {

    /*
     * The JoystickDrive class extends the CommandBase class
     * which allows it to inherit the CommandBase methods
     */

    private final DriveTrain m_drive; // Create a new DriveTrain object to hold the given DriveTrain subsystem

    public JoystickDrive(DriveTrain drive) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive); // Tell the computer that this command is using the drivetrain subsystem

        m_drive = drive; // Set the given DiveTrain equal to the Drivetrain in the command
                         // ^ the f**k? first of all why is setting a variable to another variable commented
                         // ^           second of all that makes no sense
    }

    @Override
    public void initialize() {} // Called once when the command is initially scheduled/run.

    /*
     * Called every time the scheduler runs while the command is scheduled.
     * execute() will run over and over again until isFinished() returns true
     */
    @Override
    public void execute() {
        double multiplier =
            (((RobotContainer.joystick.getThrottle() * -1) + 1) / 2); // turbo mode 
        double z = RobotContainer.joystick.getZ() / 1.5;

        if (Math.abs(z) < 0.25) {
            z = 0.0;
        }

        m_drive.drive(
            -RobotContainer.joystick.getY() * multiplier,
            RobotContainer.joystick.getX() * multiplier,
            z * multiplier,
            true,
            true
        );
    }

    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;/*
         * Since this command is the default command for the DriveTrain, it should run
         * infinitely until another command requiring the DriveTrain interrupts it.
         */
    }
}

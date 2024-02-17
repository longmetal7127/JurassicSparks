package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
//import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import static edu.wpi.first.units.Units.*;

public class ShooterTrain extends SubsystemBase {
    private CANSparkMax shooterMotorLeft = new CANSparkMax(ShooterConstants.kShooterLeftMotorCanId,
            MotorType.kBrushless);
    private CANSparkMax shooterMotorRight = new CANSparkMax(ShooterConstants.kShooterRightMotorCanId,
            MotorType.kBrushless);
    private SparkPIDController leftPID = shooterMotorLeft.getPIDController();
    private SparkPIDController rightPID = shooterMotorRight.getPIDController();
    private SysIdRoutine sysIdRoutine;

    public ShooterTrain() {
        // shooterMotorLeft.set(ShooterConstants.kSpeed);
        // shooterMotorRight.set(ShooterConstants.kSpeed);
        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    (voltage) -> shooterMotorLeft.setVoltage(voltage.in(Volts)),
                    null, // No log consumer, since data is recorded by URCL
                    this));

        leftPID.setP(.0000004119);
        leftPID.setI(0);
        leftPID.setD(0);
        leftPID.setFF(0.0021422);

        rightPID.setP(.0000004119);
        rightPID.setI(0);
        rightPID.setD(0);
        rightPID.setFF(0.0021422);

    }

    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(Direction.kForward);
    }

    public Command quasistaticBackward() {
        return sysIdRoutine.quasistatic(Direction.kReverse);
    }

    public Command dynamicForward() {
        return sysIdRoutine.dynamic(Direction.kForward);
    }

    public Command dynamicBackward() {
        return sysIdRoutine.dynamic(Direction.kReverse);
    }

    public Command setSpeed(double speed) {
        return this.runOnce(() -> {
            leftPID.setReference(speed, ControlType.kVelocity);
            rightPID.setReference(speed, ControlType.kVelocity);
        });
    }
}

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterTrain extends SubsystemBase {
    private CANSparkMax shooterMotorLeft = new CANSparkMax(ShooterConstants.kShooterLeftMotorCanId,
            MotorType.kBrushless);
    private CANSparkMax shooterMotorRight = new CANSparkMax(ShooterConstants.kShooterRightMotorCanId,
            MotorType.kBrushless);

    public ShooterTrain() {
        shooterMotorLeft.set(ShooterConstants.kSpeed); 
        shooterMotorRight.set(ShooterConstants.kSpeed);
    }

    public Command setSpeed(double speed) {
        return this.runOnce(() -> {
            shooterMotorLeft.set(speed);
            shooterMotorRight.set(speed);

        });
    }
}

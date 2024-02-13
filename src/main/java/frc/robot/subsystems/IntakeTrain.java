package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeTrain extends SubsystemBase {
    private CANSparkMax intakeMotors = new CANSparkMax(IntakeConstants.kIntakeMotorCanId, MotorType.kBrushless);

    public IntakeTrain() {
        intakeMotors.set(IntakeConstants.kSpeed);
    }

    public void setSpeed(double speed) {
        intakeMotors.set(speed);
    }
    public Command speed(double speed) {
        return this.run(()-> {
            intakeMotors.set(speed);
        });
    }
}

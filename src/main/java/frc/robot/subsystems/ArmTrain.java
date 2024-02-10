package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import monologue.Logged;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import static edu.wpi.first.units.Units.*;

public class ArmTrain extends SubsystemBase implements Logged {
    public CANSparkMax left = new CANSparkMax(ArmConstants.kLeftMotorCanId,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    public CANSparkMax right = new CANSparkMax(ArmConstants.kRightMotorCanId,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private AbsoluteEncoder m_turningEncoder;
    private SparkPIDController m_leftPID;

    private double pos = 0;
    private SysIdRoutine sysIdRoutine;

    public ArmTrain() {
        left.setSmartCurrentLimit(20);
        right.setSmartCurrentLimit(20);
        m_turningEncoder = left.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        m_leftPID = left.getPIDController();
        right.follow(left, true);
        m_turningEncoder.setPositionConversionFactor(ArmConstants.kTurningEncoderPositionFactor);

        m_leftPID.setFeedbackDevice(m_turningEncoder);

        m_leftPID.setP(ArmConstants.kTurningP);
        m_leftPID.setI(ArmConstants.kTurningI);
        m_leftPID.setD(ArmConstants.kTurningD);
        m_leftPID.setFF(ArmConstants.kTurningFF);
        m_leftPID.setOutputRange(-0.5, 0.5);
        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (voltage) -> left.setVoltage(voltage.in(Volts)),
                        null, // No log consumer, since data is recorded by URCL
                        this));

    }

    public Command setPosition(double pos) {
        return this.runOnce(() -> {
            m_leftPID.setReference(pos, ControlType.kPosition);
            this.pos = pos;
        });
    }

    public void runVolts(double volts) {
        m_leftPID.setReference(volts, ControlType.kVoltage);
    }

    public Command incrementPosition() {
        return this.runOnce(() -> {
            m_leftPID.setReference(this.pos + 10, ControlType.kPosition);
            this.pos += 10;
        });
    }

    public Command decrementPosition() {
        return this.runOnce(() -> {
            m_leftPID.setReference(this.pos - 5, ControlType.kPosition);
            this.pos -= 5;
        });
    }

    public Command quasistaticForward() {
        return sysIdRoutine.quasistatic(Direction.kForward).until(()-> {
            return m_turningEncoder.getPosition() >= 70;
        });
    }

    public Command quasistaticBackward() {
        return sysIdRoutine.quasistatic(Direction.kReverse).until(()-> {
            return m_turningEncoder.getPosition() <= 10;
        });
    }

    public Command dynamicForward() {
        return sysIdRoutine.dynamic(Direction.kForward).until(()-> {
            return m_turningEncoder.getPosition() >= 70;
        });
    }

    public Command dynamicBackward() {
        return sysIdRoutine.dynamic(Direction.kReverse).until(()-> {
            return m_turningEncoder.getPosition() <= 10;
        });
    }
 

}

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ModuleConstants;

public class ArmTrain extends SubsystemBase implements Logged {
    public CANSparkMax left = new CANSparkMax(ArmConstants.kLeftMotorCanId,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    public CANSparkMax right = new CANSparkMax(ArmConstants.kRightMotorCanId,
            com.revrobotics.CANSparkLowLevel.MotorType.kBrushless);
    private AbsoluteEncoder m_turningEncoder;
    private SparkPIDController m_leftPID;
    private SparkPIDController m_rightPID;

    public ArmTrain() {
        m_turningEncoder = left.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        m_leftPID = left.getPIDController();
        m_rightPID = right.getPIDController();
        m_leftPID.setFeedbackDevice(m_turningEncoder);
        m_rightPID.setFeedbackDevice(m_turningEncoder);

        m_leftPID.setP(ModuleConstants.kTurningP);
        m_leftPID.setI(ModuleConstants.kTurningI);
        m_leftPID.setD(ModuleConstants.kTurningD);
        m_leftPID.setFF(ModuleConstants.kTurningFF);

        m_rightPID.setP(ModuleConstants.kTurningP);
        m_rightPID.setI(ModuleConstants.kTurningI);
        m_rightPID.setD(ModuleConstants.kTurningD);
        m_rightPID.setFF(ModuleConstants.kTurningFF);

    }
    

}
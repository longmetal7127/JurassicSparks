package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ArmConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class ArmTrain extends SubsystemBase implements Logged {

  public CANSparkMax left = new CANSparkMax(
    ArmConstants.kLeftMotorCanId,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  public CANSparkMax right = new CANSparkMax(
    ArmConstants.kRightMotorCanId,
    com.revrobotics.CANSparkLowLevel.MotorType.kBrushless
  );
  private AbsoluteEncoder m_turningEncoder;
  private SparkPIDController m_leftPID;
  private ArmFeedforward feedforward = new ArmFeedforward(
    0.34728,
    0.60802,
    4.7463E-06,
    9.8036E-07
  );

  @Log.NT
  private double pos = 175;

  private SysIdRoutine sysIdRoutine;
  private double[] positions = { 13 };
  private final TrapezoidProfile m_profile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(1.75, 0.75)
  );
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
  private double kDt = 0.02; // loop time in seconds

  public ArmTrain() {
    left.restoreFactoryDefaults();

    left.setSmartCurrentLimit(60);
    right.setSmartCurrentLimit(60);
    left.setSoftLimit(SoftLimitDirection.kForward, 120);
    left.setSoftLimit(SoftLimitDirection.kReverse, 1);
    m_turningEncoder =
      left.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    m_turningEncoder.setInverted(true);
    m_leftPID = left.getPIDController();
    right.follow(left, true);
    m_turningEncoder.setPositionConversionFactor(
      ArmConstants.kTurningEncoderPositionFactor
    );

    m_turningEncoder.setVelocityConversionFactor(
      ArmConstants.kTurningEncoderPositionFactor
    );

    m_leftPID.setFeedbackDevice(m_turningEncoder);

    m_leftPID.setP(ArmConstants.kTurningP);
    m_leftPID.setI(ArmConstants.kTurningI);
    m_leftPID.setD(ArmConstants.kTurningD);
    m_leftPID.setFF(ArmConstants.kTurningFF);
    m_leftPID.setOutputRange(-1, 1);
    sysIdRoutine =
      new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
          voltage -> left.setVoltage(voltage.in(Volts)),
          null, // No log consumer, since data is recorded by URCL
          this
        )
      );
    m_leftPID.setPositionPIDWrappingEnabled(false);
    // m_leftPID.setPositionPIDWrappingMaxInput(120);
  }

  @Override
  public void periodic() {
    m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);

    m_leftPID.setReference(
      pos,
      ControlType.kPosition,
      0,
      feedforward.calculate(Math.toRadians(pos), m_setpoint.velocity)
    );
  }

  public Command setPosition(double pos) {
    return this.runOnce(() -> {
        this.pos = pos;
      });
  }

  public void setAngle(double pos) {
    m_leftPID.setReference(pos, ControlType.kPosition);
    this.pos = pos;
    m_goal = new TrapezoidProfile.State(pos, 0);
  }

  public Command incrementPosition() {
    return this.runOnce(() -> {
        m_leftPID.setReference(this.pos + 1, ControlType.kPosition);
        this.pos += 1;
      });
  }

  public Command decrementPosition() {
    return this.runOnce(() -> {
        m_leftPID.setReference(this.pos - 1, ControlType.kPosition);
        this.pos -= 1;
      });
  }

  public Command quasistaticForward() {
    return sysIdRoutine
      .quasistatic(Direction.kForward)
      .until(() -> {
        return m_turningEncoder.getPosition() >= 70;
      });
  }

  public Command quasistaticBackward() {
    return sysIdRoutine
      .quasistatic(Direction.kReverse)
      .until(() -> {
        return m_turningEncoder.getPosition() <= 10;
      });
  }

  public Command dynamicForward() {
    return sysIdRoutine
      .dynamic(Direction.kForward)
      .until(() -> {
        return m_turningEncoder.getPosition() >= 70;
      });
  }

  public Command dynamicBackward() {
    return sysIdRoutine
      .dynamic(Direction.kReverse)
      .until(() -> {
        return m_turningEncoder.getPosition() <= 10;
      });
  }
}

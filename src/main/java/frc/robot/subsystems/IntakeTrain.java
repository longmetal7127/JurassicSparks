package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
/* 
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase.ControlType;
*/
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.IntakeConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class IntakeTrain extends SubsystemBase implements Logged {

  private CANSparkMax IntakeMotorLeft = new CANSparkMax(
    IntakeConstants.kIntakeMotorCanId,
    MotorType.kBrushless
  );

  // private SparkPIDController leftPID = IntakeMotorLeft.getPIDController();
  private SysIdRoutine sysIdRoutine;
  /*private SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(
    0.12128,
    0.00012349,
    0.0010215
  );*/
  private DigitalInput beamBreak = new DigitalInput(0);
  public final Trigger hasNote = new Trigger(this::hasNote);
  public final Trigger noNote = new Trigger(beamBreak::get);

  public IntakeTrain() {
    /*
     * sysIdRoutine = new SysIdRoutine(
     * new SysIdRoutine.Config(),
     * new SysIdRoutine.Mechanism(
     * (voltage) -> IntakeMotorLeft.setVoltage(voltage.in(Volts)),
     * null, // No log consumer, since data is recorded by URCL
     * this));
     *
     * leftPID.setP(.000000091372);
     * leftPID.setI(0);
     * leftPID.setD(0);
     * // leftPID.setFF(0.0021422);
     * IntakeMotorLeft.setSmartCurrentLimit(20);
     */
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

  @Log.NT
  public boolean hasNote() {
    return !beamBreak.get();
  }

  public Command setSpeed(double speed) {
    return this.runOnce(() -> {
        // leftPID.setReference(speed, ControlType.kVelocity, 0,
        // feedForward.calculate(speed));
        IntakeMotorLeft.set(speed);
      });
  }
}

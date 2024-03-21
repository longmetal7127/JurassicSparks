package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmTrain;
import frc.robot.subsystems.DriveTrain;

public class SpeakerAim extends Command  {

  private final DriveTrain m_drive;
  private final ArmTrain m_arm;


  private PIDController turnController;

  static final double kP = 0.03;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;

  static final double kToleranceDegrees = 2.0f;
  private InterpolatingDoubleTreeMap map = new InterpolatingDoubleTreeMap();

  public SpeakerAim(DriveTrain drive, ArmTrain arm, boolean rotatebody) {
    addRequirements(arm);
    // setting up drive/arm trains
    m_drive = drive;
    m_arm = arm;
    turnController = new PIDController(kP, kI, kD);
    map.put(1.571, 207.0);
    map.put(3.02, 213.0);
    map.put(4.39, 220.0);
    map.put(1.381, 202.0);
  }

  private double distance;
  public double distanceToAngle(double dist) {
    return (-1.0105 * Math.pow(dist, 5)) +(18.8844 * Math.pow(dist, 4)) + (-135.5536 * Math.pow(dist, 3)) + (459.2128 * Math.pow(dist, 2)) + (-706.9145 * dist) + 572.6579 ;
  }


  @Override
  public void initialize() {
    turnController.reset();
    Pose3d speakerpos = new Pose3d(
      ((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? (Meters.convertFrom(53.425351, Feet)) : (0.2570046168)),
      Meters.convertFrom(18.16625, Feet),
      Meters.convertFrom(82, Inches),
      new Rotation3d()
    );
    Pose3d body = new Pose3d(
      m_drive.getPose().getX(),
      m_drive.getPose().getY(),
      0,
      new Rotation3d()
    );
    distance = body.getTranslation().getDistance(speakerpos.getTranslation());
  }

  @Override
  public void execute() {
    m_arm.setAngle(distanceToAngle(distance));
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/** TODO: Remove and add to RobotContainer or DriveTrain */
public class NavX {

  public final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  public NavX() {}
}

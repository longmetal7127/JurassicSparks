package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class NavX extends SubsystemBase {
    private AHRS ahrs;

    public NavX() {
        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.reset();
        ahrs.calibrate();
    }

    public double getPitch() {
        return ahrs.getPitch();
    }

    public double getYaw() {
        return ahrs.getYaw();
    }

    public void resetYaw() {
        ahrs.zeroYaw();
    }

    public Rotation2d getRotation2d() {
        return ahrs.getRotation2d();
    }
}

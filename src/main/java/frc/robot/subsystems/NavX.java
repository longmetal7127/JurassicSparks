package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

public class NavX {

    public final AHRS ahrs = new AHRS(SPI.Port.kMXP);

    public NavX() {
        ahrs.reset();

        // ahrs.calibrate();
    }

    /*
     * Deprecated: Use .ahrs.getPitch()
     */
    public double getPitch() {
        return ahrs.getPitch();
    }

    /*
     * Deprecated: Use .ahrs.getYaw()
     */
    public double getYaw() {
        return ahrs.getYaw();
    }

    /*
     * Deprecated: Use .ahrs.zeroYaw()
     */
    public void resetYaw() {
        ahrs.zeroYaw();
    }

    /*
     * Deprecated: Use .ahrs.getRate()
     */
    public double getRate() {
        return ahrs.getRate();
    }

    /*
     * Deprecated: Use .ahrs.getRotation2d()
     */

    public Rotation2d getRotation2d() {
        return  ahrs.getRotation2d();
    }
}

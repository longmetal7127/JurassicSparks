package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmTrain;
import frc.robot.subsystems.DriveTrain;
import monologue.Logged;
import monologue.Annotations.Log;

/*
 * FYI this is all theoretical stuff; not sure if it works yet
 * 
 * 
 * (it probably will)
 */
public class SpeakerAim extends Command implements Logged {
    // necessary/looping stuff
    private final DriveTrain m_drive;
    private final ArmTrain m_arm;
    private boolean m_rotatebody;
    private int loop = 1;
    private int repetitions = 10;

    // positioning
    private Pose3d speakerpos;
    @Log.NT
    private Pose3d body;
    private Pose3d shootpos;
    private Pose3d axelpos;

    // tuning stuff for distances, positions, constants, or global vars
    private double arml = 2.16279528;
    private double pointaxeldis = 0.6666667;
    private double bumph = 0.7468586;
    private double shootrot = 59;
    private double gravity = 32f;
    private double angle = -1;
    private double velocity = 50;
    private double xzdiff;
    private double thetapos;
    private double thetaneg;
    private double shootflrlength;
    private double ydiff;
    private double yaw;

    public SpeakerAim(DriveTrain drive, ArmTrain arm, boolean rotatebody) {
        // setting up drive/arm trains
        addRequirements(drive);
        m_drive = drive;
        m_arm = arm;
        m_rotatebody = rotatebody;

        // setting up positional vectors
    }

    @Override
    public void initialize() {
        speakerpos = new Pose3d(
                53.425351,
                18.16625,
                6.666666667,
                new Rotation3d());
        body = new Pose3d(
                Feet.convertFrom(m_drive.getPose().getX(), Meters),
                Feet.convertFrom(m_drive.getPose().getY(), Meters),
                0.2270997,
                new Rotation3d());

        // finding necessary body rotation
        yaw = 180 - (Math.toDegrees(Math.atan(speakerpos.getY()-body.getY())) / (speakerpos.getX()-body.getX()));
        double opyaw = yaw - 180;
        if (m_rotatebody) {
            m_drive.setYaw(opyaw);
        }

        // not sure if i need this but its here anyway
        body.rotateBy(new Rotation3d(0, 0, opyaw));

        // setting up axel pos
        axelpos = new Pose3d(
                (body.getX() - (Math.cos(Math.toRadians(yaw)) * pointaxeldis)),
                (body.getY() - (Math.sin(Math.toRadians(yaw)) * pointaxeldis)),
                (bumph + body.getZ()),
                new Rotation3d());
        shootpos = axelpos;
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return (loop >= repetitions);
    }
}
/*
 * float yaw = -(Mathf.Rad2Deg * Mathf.Atan((speakerpos.position.z - body.transform.position.z) / (speakerpos.position.x - body.transform.position.x)));
        body.Rotate(0, 0, yaw);

        Vector3 axelpos = new(
            (body.position.x + (Mathf.Cos(Mathf.Deg2Rad * yaw) * pointaxeldis)),
            bumph,
            (body.position.z + (Mathf.Sin(Mathf.Deg2Rad * yaw) * pointaxeldis))
        );
        axelpos = transform.position;

        float sidedis = Vector3.Distance(axelpos, speakerpos.position);
        float siderot = Mathf.Rad2Deg * Mathf.Atan(Mathf.Sqrt(Mathf.Pow(speakerpos.position.x - axelpos.x, 2) + Mathf.Pow(speakerpos.position.z - axelpos.z, 2)) / (speakerpos.position.y - axelpos.y));

        float angle = 90 - (180 - (Mathf.Rad2Deg * Mathf.Asin((Mathf.Sin(Mathf.Deg2Rad * shootrot) * arml) / sidedis)) - shootrot - siderot);
        float shootflrlength = (Mathf.Cos(Mathf.Deg2Rad * angle) * arml);

        transform.Rotate(0, 0, angle);
        Vector3 shootpos = new(
            (axelpos.x - Mathf.Cos(Mathf.Deg2Rad * yaw) * shootflrlength),
            (axelpos.y + Mathf.Sin(Mathf.Deg2Rad * angle) * arml),
            (axelpos.z + Mathf.Sin(Mathf.Deg2Rad * yaw) * shootflrlength)
        );
        Object newnote = Instantiate(note, shootpos, Quaternion.Euler(-90, 90 + yaw, 0));
        //float vectflrlenght = Mathf.Cos(angle) * newvel;
        //print(newvel);
        newnote.GetComponent<Transform>().Rotate(angle + 180 - shootrot, 0, 0);
        //newnote.GetComponent<move>().setVector(new(Mathf.Cos(Mathf.Deg2Rad * yaw) * vectflrlenght, Mathf.Sin(Mathf.Deg2Rad * angle) * newvel, Mathf.Sin(Mathf.Deg2Rad * yaw) * vectflrlenght));
        newnote.GetComponent<Transform>().Rotate(0, 0, 0);
 */

 /*
  * // the differences in position
        xzdiff = Math.sqrt(
                Math.pow(speakerpos.getX()-shootpos.getX(), 2) + Math.pow(speakerpos.getY()-shootpos.getY(), 2));
        ydiff = speakerpos.getZ() - shootpos.getZ();

        // funky math stuff (dw about it its flawless)
        thetapos = Math.atan((Math.pow(velocity, 2) + Math.sqrt(
                Math.pow(velocity, 4) - gravity * (gravity * Math.pow(xzdiff, 2) + 2 * ydiff * Math.pow(velocity, 2))))
                / (gravity * xzdiff));
        thetaneg = Math.atan((Math.pow(velocity, 2) - Math.sqrt(
                Math.pow(velocity, 4) - gravity * (gravity * Math.pow(xzdiff, 2) + 2 * ydiff * Math.pow(velocity, 2))))
                / (gravity * xzdiff));

        if (Double.NaN == thetaneg || Double.NaN == thetapos) {
            // break loop if angle is NAN (aka the velocity isnt great enough to get the
            // disk to the speaker, no matter the angle)
            angle = -1;
            loop = repetitions;
        } else {
            // finding the best suit for the angle, pos or neg
            if (thetapos > thetaneg) {
                System.out.println(Math.toDegrees(thetaneg));
                angle = shootrot - (Math.toDegrees(thetaneg));
            } else if (thetapos < thetaneg) {
                System.out.println(Math.toDegrees(thetapos));
                angle = shootrot - (Math.toDegrees(thetapos));
            }

            // find the new position the disk is shot from for greater accuracy in further
            // repetitions
            shootflrlength = (Math.cos(Math.toRadians(angle)) * arml);
            shootpos = new Pose3d(
                    (axelpos.getX() - Math.cos(Math.toRadians(yaw)) * shootflrlength),
                    (axelpos.getY() - Math.sin(Math.toRadians(yaw)) * shootflrlength),
                    (axelpos.getZ() + Math.sin(Math.toRadians(angle)) * arml),
                    new Rotation3d()
                );
        }

        // set the angle of the arm at the end of the loop
        if (angle != -1 && loop >= repetitions) {
            System.out.println("body x "+body.getX());
            System.out.println("body y "+body.getY());
            System.out.println("yaw " + yaw);
            System.out.println("axelpos x "+axelpos.getX());
            System.out.println("axelpos y "+axelpos.getY());
            System.out.println("angle "+angle);
            m_arm.setAngle(175 + Math.min(Math.max(angle, 0),90));
        }

        // increment time
        loop++;
  */
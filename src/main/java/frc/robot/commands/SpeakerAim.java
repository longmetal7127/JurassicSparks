package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmTrain;
import frc.robot.subsystems.DriveTrain;
/*
 * FYI this is all theoretical stuff; not sure if it works yet
 * 
 * 
 * (it probably will)
 */
public class SpeakerAim extends Command {
    //necessary/looping stuff
    private final DriveTrain m_drive;
    private final ArmTrain m_arm;
    private boolean m_rotatebody;
    private int loop = 1;
    private int repetitions = 5;

    //positioning
    private Pose3d speakerpos;
    private Pose3d body;
    private Pose3d shootpos;
    private Pose3d axelpos;

    //tuning stuff for distances, positions, constants, or global vars
    private double arml = 2.16279528;
    private double pointaxeldis = 0.6700001;
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
        //setting up drive/arm trains
        addRequirements(drive);
        m_drive = drive;
        m_arm = arm;
        m_rotatebody = rotatebody;

        //setting up positional vectors
        speakerpos = new Pose3d(
            8.77375, //this stuff needs to be tuned
            0.984962,
            6.953999,
            new Rotation3d()
        );
        body = new Pose3d(
            drive.getPose().getX(),
            drive.getPose().getY(),
            0.2270997,
            new Rotation3d()
        );

        //finding necessary body rotation
        yaw = 180 - (Math.toDegrees(Math.atan((body.getY() - speakerpos.getY()) / (body.getX() - speakerpos.getX()))));
        double opyaw = yaw - 180;
        if (m_rotatebody) {
            m_drive.setYaw(
                opyaw
            );
        }

        //not sure if i need this but its here anyway
        body.rotateBy(new Rotation3d(0,0,opyaw));

        //setting up axel pos
        axelpos = new Pose3d(
            (body.getX() + (Math.cos(Math.toRadians(yaw)) * pointaxeldis)),
            (body.getY() + (Math.sin(Math.toRadians(yaw)) * pointaxeldis)),
            (bumph + body.getZ()),
            new Rotation3d()
        );
        shootpos = axelpos;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // the differences in position
        xzdiff = Math.sqrt(Math.pow(shootpos.getX() - speakerpos.getX(), 2) + Math.pow(shootpos.getY() - speakerpos.getY(), 2));
        ydiff = speakerpos.getZ() - shootpos.getZ();

        // funky math stuff (dw about it its flawless)
        thetapos = Math.atan((Math.pow(velocity, 2) + Math.sqrt(
            Math.pow(velocity, 4) - gravity * (gravity * Math.pow(xzdiff, 2) + 2 * ydiff * Math.pow(velocity, 2))))
            / (gravity * xzdiff)
        );
        thetaneg = Math.atan((Math.pow(velocity, 2) - Math.sqrt(
            Math.pow(velocity, 4) - gravity * (gravity * Math.pow(xzdiff, 2) + 2 * ydiff * Math.pow(velocity, 2))))
            / (gravity * xzdiff)
        );

        if (Double.NaN == thetaneg || Double.NaN == thetapos) {
            // break loop if angle is NAN (aka the velocity isnt great enough to get the
            // disk to the speaker, no matter the angle)
            angle = -1;
            loop = repetitions;
        } else {
            // finding the best suit for the angle, pos or neg
            if (thetapos > thetaneg) {
                angle = shootrot - (Math.toDegrees(thetaneg));
            } else if (thetapos < thetaneg) {
                angle = shootrot - (Math.toDegrees(thetapos));
            }

            //find the new position the disk is shot from for greater accuracy in further repetitions
            shootflrlength = (Math.cos(Math.toRadians(angle)) * arml);
            shootpos = new Pose3d(
                (axelpos.getX() - Math.cos(Math.toRadians(yaw)) * shootflrlength),
                (axelpos.getY() - Math.sin(Math.toRadians(yaw)) * shootflrlength),
                (axelpos.getZ() + Math.sin(Math.toRadians(angle)) * arml),
                new Rotation3d()
            );
        }

        //set the angle of the arm at the end of the loop
        if (angle != -1 && loop >= repetitions) {
            m_arm.setAngle(
                angle
            );
        }

        //increment time
        loop++;
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return (loop >= repetitions);
    }
}
package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ModuleConstants;
import monologue.Logged;

public class ClimbTrain extends XboxController{
    public ClimbTrain() {
        if (getXButtonPressed() == true){
            System.out.println("test");
        }
        else{
            System.out.println("fail");
        }
    }
}

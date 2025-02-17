package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.Map;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

//import edu.wpi.first.units.measure.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeSubsystem extends SubsystemBase{
    private SparkMax m_AlgaeIntake; 
    private SparkMaxConfig c_AlgaeIntake;
    // AbsoluteEncoder e_AlgaeEncoder = m_AlgaeIntake.getAbsoluteEncoder();
    
    public AlgaeIntakeSubsystem() {
        m_AlgaeIntake = new SparkMax(AlgaeConstants.kAlgae, MotorType.kBrushless);
        m_AlgaeIntake.getOutputCurrent();
    }

/* .getCurrent acts both as a getter ans setter*/
    public void Intake(){
        m_AlgaeIntake.set(0.65); // puts the speed of the motor 
    }

    public void IntakeStop(){
        m_AlgaeIntake.set(0); // puts the speed of the motor to zero 
    }

    public void Outtake(){
        m_AlgaeIntake.set(-0.65); // puts the speed of the motor to go backwards 
    }

}

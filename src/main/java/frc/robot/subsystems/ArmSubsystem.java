package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import java.util.Map;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
        private SparkMax m_Arm; 
        private SparkMaxConfig c_Arm;
        AbsoluteEncoder e_ArmEncoder = m_Arm.getAbsoluteEncoder();

    public ArmSubsystem() {
        m_Arm = new SparkMax(ArmConstants.kArm, MotorType.kBrushless);
    }
  
    public void armDown(){
       if (e_ArmEncoder.getPosition() != 0){ //tbd - not sure what the positions are
         m_Arm.set(.7);                //TODO: add numbers to .getPosition
       }
       else if (e_ArmEncoder.getPosition() == 0){
        m_Arm.set(0); 
       }    
    }

    public void armUp(){
        if (e_ArmEncoder.getPosition() != 0){
          m_Arm.set(.7); 
        }
        else if (e_ArmEncoder.getPosition() == 0){
         m_Arm.set(0); 
        }   
     }
}
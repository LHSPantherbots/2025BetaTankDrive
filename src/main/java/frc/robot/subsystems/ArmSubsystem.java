package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
 private final WPI_TalonSRX m_Arm = new  WPI_TalonSRX(1);
    
    public ArmSubsystem() {

    }

    public void armDown(){
       m_Arm.set(0.1);
    }

    public void armUp(){
      m_Arm.set(-0.1);
     }

    public void armStop(){
      m_Arm.set(0);
    }
}
package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIntakeSubsystem extends SubsystemBase{
    
    //TODO: get the parameters proper
    private DCMotor m_AlgaeIntake = new DCMotor(0, 0, 0, 0, 0, 0); //uses CIM motors
    
    public AlgaeIntakeSubsystem() {
        
    }

/* .getCurrent acts both as a getter ans setter*/
    public void Intake(){
        m_AlgaeIntake.getCurrent(0.7); // puts the speed of the motor 
    }

    public void Stop(){
        m_AlgaeIntake.getCurrent(0); // puts the speed of the motor to zero 
    }

    public void Outtake(){
        m_AlgaeIntake.getCurrent(-0.7); // puts the speed of the motor to go backwards 
    }

}

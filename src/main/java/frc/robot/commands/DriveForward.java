// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

/** An example command that uses an example subsystem. */
public class DriveForward extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_driveTrain;
  long i;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveForward(Drivetrain drive) {
    this.m_driveTrain = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        m_driveTrain.drive(0.5, 0.0); //TODO: TEST THAT THIS ACTUALLY WORKS 
        //WithTimeout was removed because the method in drive just throws an error
        //if we want to use WithTimeout, we have to implement it.
        // the following is an attempt to make a solution was meant to implement
        i++;
        
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(i == 300){
      return true;
    }
    else{
      return false;
    }
  }

// public static Command RunCommand(Drivetrain drive) {
//     // TODO: Auto-generated method stub
   
// }
  
}

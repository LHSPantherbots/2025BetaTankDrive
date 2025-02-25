// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.Autos;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drive = new Drivetrain();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final AlgaeIntakeSubsystem intakeAlgae = new AlgaeIntakeSubsystem();

   

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

  //SmartDashboard.putData("Auto", new RunCommand(() -> autoDriveForward.execute()));


    // Configure the trigger bindings
    configureBindings();

    drive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(() -> drive.drive(-driverController.getLeftY(),
                                          driverController.getRightX() 
                                          ),drive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(drive::exampleCondition)
    //     .onTrue(new ExampleCommand(drive));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.)
    // driverController.b().whileTrue(drive.exampleMethodCommand());

    // driverController.a().onTrue((new RunCommand(() -> drive.maxMotionPosition(), drive)));
    // driverController.x().onTrue((new InstantCommand(() -> drive.drive(0,0), drive)));
    // driverController.povUp().onTrue(new InstantCommand(() -> drive.setLeftSetpoint(1.0), drive));
    // driverController.povDown().onTrue(new InstantCommand(() -> drive.setLeftSetpoint(0.0), drive));
    

    operatorController.y().onTrue((new InstantCommand(() -> arm.armUp(), arm)));
    operatorController.x().onTrue((new InstantCommand(() -> arm.armDown(), arm)));
    operatorController.a().onTrue((new InstantCommand(() -> arm.armStop(), arm)));
    operatorController.rightTrigger().whileTrue(new InstantCommand(() -> intakeAlgae.Intake(), intakeAlgae)).whileFalse(new InstantCommand(() -> intakeAlgae.IntakeStop(), intakeAlgae));
    operatorController.leftTrigger().whileTrue(new InstantCommand(() -> intakeAlgae.Outtake(), intakeAlgae)).whileFalse(new InstantCommand(() -> intakeAlgae.IntakeStop(), intakeAlgae));
    



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * 
   */
  public Command getAutonomousCommand() {
     // An example command will be run in autonomous
     // return Autos.exampleAuto(drive);

     //return new RunCommand(() -> autoDriveForward.execute());//.andThen(autoDriveForward.end(false));
     return new RunCommand(()->drive.drive(.5,.0),drive).withTimeout(2.0);

 
 }
}


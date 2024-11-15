// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    private final int rightDriveCANid = 19;
    private final int rightFollowerCANid = 3;
    private final int leftDriveCANid = 13;
    private final int leftFollowerCANid = 1;
    private final double DRIVE_GEAR_RATIO = (50.0/14.0)*(48.0/16.0);
    private final double WHEEL_CIRCUMFERENCE_METER = Math.PI*6.0*25.4/1000.0;
    private final SparkMax rightDrive;
    private final SparkMax rightFollower;
    private final SparkMax leftDrive;
    private final SparkMax leftFollower;

    private final RelativeEncoder rightDrivingEncoder;
    private final RelativeEncoder leftDrivingEncoder;

    private double leftSetpoint = 0.0;


    private final SparkMaxConfig rightConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightFollowConfig = new SparkMaxConfig();
    private final SparkMaxConfig leftConfig = new SparkMaxConfig();
    private final SparkMaxConfig leftFollowConfig = new SparkMaxConfig();


    private final SparkClosedLoopController rightController;
    private final SparkClosedLoopController leftController;

    //private final DifferentialDrive diffDrive;
    

  public DriveSubsystem() {
    rightDrive = new SparkMax(rightDriveCANid, MotorType.kBrushless);
    rightFollower = new SparkMax(rightFollowerCANid, MotorType.kBrushless);

    leftDrive = new SparkMax(leftDriveCANid, MotorType.kBrushless);
    leftFollower = new SparkMax(leftFollowerCANid, MotorType.kBrushless);

   // diffDrive = new DifferentialDrive(leftDrive, rightDrive);


    rightDrivingEncoder = rightDrive.getEncoder();
    leftDrivingEncoder = leftDrive.getEncoder();

    rightController = rightDrive.getClosedLoopController();
    leftController = leftDrive.getClosedLoopController();


    rightConfig
                  .idleMode(IdleMode.kBrake)
                  .smartCurrentLimit(40)
                  .inverted(true);
    rightConfig.encoder
                  .positionConversionFactor(1.0/DRIVE_GEAR_RATIO*WHEEL_CIRCUMFERENCE_METER) // meters
                  .velocityConversionFactor(1.0/DRIVE_GEAR_RATIO*WHEEL_CIRCUMFERENCE_METER/60.0);//meters/sec
//                  .inverted(false);
    rightConfig.closedLoop
                  .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                  .p(0.02)
                  .i(0)
                  .d(0.0)
                  .velocityFF(0.0)
                  .maxOutput(1.0)
                  .minOutput(-1.0)
                  .maxMotion
                    .maxAcceleration(1.0)
                    .maxVelocity(1.0)
                    .allowedClosedLoopError(.1) //degrees
                    ;


    rightFollowConfig
                  .idleMode(IdleMode.kBrake)
                  .smartCurrentLimit(40)
                  .follow(rightDrive)
                  .inverted(false); //will automatically invert with leader only needs to be changed if turning opposite of leader
                  
   

      leftConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40)
                    .inverted(false);
      leftConfig.encoder
                    .positionConversionFactor(1.0/DRIVE_GEAR_RATIO*WHEEL_CIRCUMFERENCE_METER) //meters
                    .velocityConversionFactor(1.0/DRIVE_GEAR_RATIO*WHEEL_CIRCUMFERENCE_METER/60.0); //meters/sec
      //              .inverted(false);
      leftConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .p(50.0)//0.5)
                    .i(0)
                    .d(0.0)
                    .velocityFF(0.0)
                    .maxOutput(1.0)
                    .minOutput(-1.0)
                    .maxMotion
                      .maxAcceleration(1.49/2.0)
                      .maxVelocity(1.49)//2000.0)
                      .allowedClosedLoopError(.1) //degrees
                      ;

      leftFollowConfig
                      .idleMode(IdleMode.kBrake)
                      .smartCurrentLimit(40)
                      .follow(leftDrive)
                      .inverted(false); //will automatically invert with leader only needs to be changed if turning opposite of leader

    

    rightDrive.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightFollower.configure(rightFollowConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftDrive.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftFollower.configure(leftFollowConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



    zeroEncoders();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state0, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Right Encoder Position", rightDrivingEncoder.getPosition());
    SmartDashboard.putNumber("Left Encoder Position", leftDrivingEncoder.getPosition());
    SmartDashboard.putNumber("Right Velocity", rightDrivingEncoder.getVelocity());
    SmartDashboard.putNumber("Left Velocity", leftDrivingEncoder.getVelocity());
    SmartDashboard.putNumber("Left Setpoint", leftSetpoint);
    SmartDashboard.putNumber("Left Motor RPS", leftDrivingEncoder.getVelocity()*DRIVE_GEAR_RATIO*60.0/WHEEL_CIRCUMFERENCE_METER);
    
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //public void drive(double move, double turn){
  //  diffDrive.arcadeDrive(move, turn);
  //}

  public void drive(double move, double turn){
    leftDrive.set(move);
  }

  public void zeroEncoders(){
    rightDrivingEncoder.setPosition(0.0);
    leftDrivingEncoder.setPosition(0.0);
  }


  public void setLeftSetpoint(double value){
    this.leftSetpoint=value;
  }

  public void maxMotionPosition(){
    leftController.setReference(leftSetpoint, ControlType.kMAXMotionPositionControl);

  }
}

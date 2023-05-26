// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenixpro.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class SwerveModule extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final TalonFX m_turnMotor;
  private final TalonFX m_driveMotor;
  private final CANcoder m_CANcoder;
  private final PIDController SwervePID;

  public SwerveModule() {
    m_turnMotor = new TalonFX(RobotMap.Swerve.Module0.turnMotorID);
    m_driveMotor = new TalonFX(RobotMap.Swerve.Module0.driveMotorID);
    m_CANcoder = new CANcoder(RobotMap.Swerve.Module0.CANcoderID);

    SwervePID = new PIDController(Constants.SwerveModule.SwerveP, Constants.SwerveModule.SwerveI, Constants.SwerveModule.SwerveD);

  }

  public void setSpeed(){
    m_driveMotor.set(ControlMode.PercentOutput, SwervePID.calculate(0,0));
  }
  //pid needs setpoint and measurment

  public void getAngle(){

  }

  public void setAngle(){
    
  }

  public void getPosition(){

  }

  //do the get stuff and set stuff
  //cancoder offset thing because 0 is not 0
  // do all the invert stuff
  //Look at old code to figure out how stuff works because this is confusing

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
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
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}

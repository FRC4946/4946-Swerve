package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

import com.ctre.phoenix6.controls.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.Utils.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase {
  
  private final TalonFX m_turnMotor;
  private final TalonFX m_driveMotor;
  private final CANcoder m_CANCoder;
  private final Rotation2d turnMotorOffset;

  private final PIDController turnPID;

  private final SwerveModuleState driveForward;
  private final Rotation2d defaultAngle;


  public SwerveModule(SwerveModuleConstants swerveModConstants) {
    m_driveMotor = new TalonFX(swerveModConstants.driveMotorID, RobotMap.CAN.CANivoreID);
    m_turnMotor = new TalonFX(swerveModConstants.turnMotorID, RobotMap.CAN.CANivoreID);
    m_CANCoder = new CANcoder(swerveModConstants.CANCoderID, RobotMap.CAN.CANivoreID);
    this.turnMotorOffset = swerveModConstants.angleOffset;

    turnPID = new PIDController(Constants.Swerve.turnKP, Constants.Swerve.turnKI, Constants.Swerve.turnKD);
    turnPID.enableContinuousInput(0, 360);

    defaultAngle = new Rotation2d(0);
    driveForward = new SwerveModuleState(Constants.Swerve.maxSpeed, defaultAngle);
  }

  public void setSpeed(SwerveModuleState state, boolean openLoop){
    if(openLoop){
      m_driveMotor.setControl(new VelocityDutyCycle(state.speedMetersPerSecond));
    } else {
      double desiredSpeed = state.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      m_driveMotor.setControl(new DutyCycleOut(desiredSpeed));
    }
  }

  public void setSpeed(double speed, boolean openLoop){
    if(openLoop){
      m_driveMotor.setControl(new VelocityDutyCycle(speed));
    } else {
      double desiredSpeed = speed / Constants.Swerve.maxSpeed;
      m_driveMotor.setControl(new DutyCycleOut(desiredSpeed));
    }
  }

  public void setAngle(SwerveModuleState state){
    m_turnMotor.setControl(new PositionDutyCycle(state.angle.getDegrees()));
  }

  public void setAngle(Rotation2d angle){
    m_turnMotor.setControl(new PositionDutyCycle(angle.getDegrees()));
  }

  public void setAngle(double speed){
    m_turnMotor.setControl(new DutyCycleOut(speed));
  }

  public void setState(SwerveModuleState desiredState){
    setAngle(desiredState);
    setSpeed(desiredState, false);
  }

  public double getAngle(){
    return m_CANCoder.getAbsolutePosition().getValue() - turnMotorOffset.getDegrees();
  }
  
  public double getSpeed(boolean returnInMPS){
    if(returnInMPS){
      double mPS = (m_driveMotor.getRotorVelocity().getValue() *10*Constants.Swerve.wheelCircumference)/2048;
      return mPS;
    } else {
    return m_driveMotor.getRotorVelocity().getValue();
    }
  }

  @Override
  public void periodic() {

  }
}
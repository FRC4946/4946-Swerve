package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Utils.SwerveModuleConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
public class SwerveModule extends SubsystemBase {
  
  private final TalonFX m_turnMotor;
  private final TalonFX m_driveMotor;
  private final CANCoder m_CANCoder;
  private final Rotation2d turnMotorOffset;

  private final PIDController turnPID;
  private final PIDController drivePID;

  private final SwerveModuleState driveForward;
  private final Rotation2d defaultAngle;

  public SwerveModule(SwerveModuleConstants swerveModConstants) {
    m_driveMotor = new TalonFX(swerveModConstants.driveMotorID);
    m_turnMotor = new TalonFX(swerveModConstants.turnMotorID);
    m_CANCoder = new CANCoder(swerveModConstants.CANCoderID);
    this.turnMotorOffset = swerveModConstants.angleOffset;

    turnPID = new PIDController(Constants.Swerve.turnKP, Constants.Swerve.turnKI, Constants.Swerve.turnKD);
    drivePID = new PIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);
    turnPID.enableContinuousInput(0, 360);

    defaultAngle = new Rotation2d(0);
    driveForward = new SwerveModuleState(Constants.Swerve.maxSpeed, defaultAngle);
  }

  public void setSpeed(SwerveModuleState state, boolean openLoop){
    if(openLoop){
      double desiredSpeed = drivePID.calculate(getSpeed(true), state.speedMetersPerSecond);
      m_driveMotor.set(ControlMode.PercentOutput, desiredSpeed);
    } else {
      double desiredSpeed = state.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      m_driveMotor.set(ControlMode.PercentOutput, desiredSpeed);
    }
  }

  public void setSpeed(double speed, boolean openLoop){
    if(openLoop){
      double desiredSpeed = drivePID.calculate(getSpeed(true), speed);
      m_driveMotor.set(ControlMode.PercentOutput, desiredSpeed);
    } else {
      double desiredSpeed = speed / Constants.Swerve.maxSpeed;
      m_driveMotor.set(ControlMode.PercentOutput, desiredSpeed);
    }
  }

  public void setAngle(SwerveModuleState state){
    m_turnMotor.set(ControlMode.PercentOutput, turnPID.calculate(getAngle(), state.angle.getDegrees()));
  }

  public void setAngle(double angle){
    m_turnMotor.set(ControlMode.PercentOutput, turnPID.calculate(getAngle(), angle));
  }

  public void setState(SwerveModuleState desiredState){
    setAngle(desiredState);
    setSpeed(desiredState, false);
  }

  public double getAngle(){
    return m_CANCoder.getAbsolutePosition() - turnMotorOffset.getDegrees();
  }
  
  public double getSpeed(boolean returnInMPS){
    if(returnInMPS){
      double mPS = (m_driveMotor.getSelectedSensorVelocity()*10*Constants.Swerve.wheelCircumference)/2048;
      return mPS;
    } else {
    return m_driveMotor.getSelectedSensorVelocity();
    }
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}

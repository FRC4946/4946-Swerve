package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
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

  public SwerveModule(SwerveModuleConstants swerveModConstants) {
    m_driveMotor = new TalonFX(swerveModConstants.driveMotorID);
    m_turnMotor = new TalonFX(swerveModConstants.turnMotorID);
    m_CANCoder = new CANCoder(swerveModConstants.CANCoderID);
    this.turnMotorOffset = swerveModConstants.angleOffset;

    turnPID = new PIDController(Constants.Swerve.turnKP, Constants.Swerve.turnKI, Constants.Swerve.turnKD);
    drivePID = new PIDController(Constants.Swerve.driveKP, Constants.Swerve.driveKI, Constants.Swerve.driveKD);
    turnPID.enableContinuousInput(0, 360);
  }

  public void setSpeed(SwerveModuleState state, boolean openLoop){
    if(openLoop){
      double desiredSpeed = state.speedMetersPerSecond;
      m_driveMotor.set(ControlMode.PercentOutput, desiredSpeed);
    } else {
      double desiredSpeed = state.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      m_driveMotor.set(ControlMode.PercentOutput, desiredSpeed);
    }
  }

  public void setAngle(SwerveModuleState state){
    m_turnMotor.set(ControlMode.PercentOutput, turnPID.calculate(getAngle() - turnMotorOffset.getDegrees(), state.angle.getDegrees()));
  }

  public void setState(SwerveModuleState desiredState){
    setAngle(desiredState);
    setSpeed(desiredState, false);
  }

  public double getAngle(){
    return m_CANCoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    
  }
}

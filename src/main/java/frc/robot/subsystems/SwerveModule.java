package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
public class SwerveModule extends SubsystemBase {
  
  private final TalonFX m_turnMotor;
  private final TalonFX m_driveMotor;
  private final CANCoder m_CANCoder;
  private final double turnMotorOffset;

  private final PIDController turnPID;

  public SwerveModule() {
    m_turnMotor = new TalonFX(RobotMap.Swerve.Mod0.driveMotorID);
    m_driveMotor = new TalonFX(RobotMap.Swerve.Mod0.turnMotorID);
    m_CANCoder = new CANCoder(RobotMap.Swerve.Mod0.CANCoderID);
    turnMotorOffset = Constants.Swerve.mod0TurnOffset;

    turnPID = new PIDController(Constants.Swerve.turnKP, Constants.Swerve.turnKI, Constants.Swerve.turnKD);
    turnPID.enableContinuousInput(0, 360);
  }

  public void setSpeed(double speed){
    m_driveMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setAngle(double desiredAngle){
    m_turnMotor.setSelectedSensorPosition(turnPID.calculate(getAngle(), desiredAngle));
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

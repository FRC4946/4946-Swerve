package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj.XboxController;

public class DashboardSwerDrive extends CommandBase {

  private final SwerveModule mod0;
  private final CommandXboxController driver;
  
  public DashboardSwerDrive(SwerveModule mod0, CommandXboxController m_driverController) {
    this.mod0 = mod0;
    this.driver = m_driverController;
  
    addRequirements(mod0);
  }

  @Override
  public void initialize() {
  
  }

  @Override
  public void execute() {
    double speed = driver.getRawAxis(XboxController.Axis.kLeftY.value);
    double rotate = driver.getRawAxis(XboxController.Axis.kRightX.value);
    mod0.setSpeed(speed, false);
    mod0.setAngle(rotate);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
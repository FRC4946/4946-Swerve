package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveModule;

public class DashboardSwerDrive extends CommandBase {

  private SwerveModule mod0;
  /** Creates a new DashboardSwerDrive. */
  public DashboardSwerDrive(SwerveModule mod0) {
    this.mod0 = mod0;
  }

  @Override
  public void initialize() {
  
  }

  @Override
  public void execute() {
    double moduleSpeed = SmartDashboard.getNumber("Module Speed", 0);
    double moduleAngle = SmartDashboard.getNumber("Module Angle", 0);
    mod0.setSpeed(moduleSpeed, true);
    mod0.setAngle(moduleAngle);
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
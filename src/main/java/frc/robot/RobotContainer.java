package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DashboardSwerDrive;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final SwerveModule mod0;

  private final CommandXboxController m_driverController;

  public RobotContainer() {
    mod0 = new SwerveModule(RobotMap.Swerve.Mod0.swerveMod);
    m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

    configureBindings();
  }
  
  private void configureBindings() {
    mod0.setDefaultCommand(new DashboardSwerDrive(mod0, m_driverController));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}

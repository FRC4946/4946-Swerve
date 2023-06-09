package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Utils.SwerveModuleConstants;

public class RobotMap {
    
    public static final class Swerve {
        public static final class Mod0 {
            public static final int driveMotorID = 0;
            public static final int turnMotorID = 1;
            public static final int CANCoderID = 0;
            public static final Rotation2d angleOffset = Constants.Swerve.swerveMod1AngleOffset;
            public static final SwerveModuleConstants swerveMod = new SwerveModuleConstants(driveMotorID, turnMotorID, CANCoderID, angleOffset);
        }
    }
}

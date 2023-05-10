package frc.robot.Utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int turnMotorID;
    public final int CANCoderID;
    public final Rotation2d angleOffset;

    public SwerveModuleConstants(int driveMotorID, int turnMotorID, int CANCoderID, Rotation2d angleOffset){
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.CANCoderID = CANCoderID;
        this.angleOffset = angleOffset;
    }
}

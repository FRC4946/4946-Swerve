// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class Swerve{
    public static final double mod0TurnOffset = 0;

    public static final double turnKP = 1;
    public static final double turnKI = 0;
    public static final double turnKD = 0;

    public static final double driveKP = 1;
    public static final double driveKI = 0;
    public static final double driveKD = 0;

    public static final Rotation2d swerveMod1AngleOffset = new Rotation2d(Math.toRadians(0));

    public static final double maxSpeed = 4;

    public static final double wheelDiameter = 0; //this value should be in meters
    public static final double wheelCircumference = wheelDiameter * Math.PI;
  }
}

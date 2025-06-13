// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

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
    public static final int kOperatorControllerPort = 1;
  }
  public static class ElevatorConstants {
    public static final int kMotorId = 1;
    public static final int kSlaveId = 2;
    public static final int kDigitalPort = 0;
    public static final String kCanBus = "rio";
    public static final TalonFXConfiguration kTalonFCFactoryDefaults = new TalonFXConfiguration();
    public static final double kLevel1 = 1000;
    public static final double KLevel2 = 2000;
    public static final double KLevel3 = 3000;
    public static final double KLoadit = 2500;
}

}

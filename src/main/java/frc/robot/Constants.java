// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce ity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = .05;
  }
  

  public static final double maxSpeed = Units.feetToMeters(10);

  //"absoluteEncoderOffset": 0.81298828125, 
  
 // Elevator PID Constants (TUNE THESE)
 public static final double ELEVATOR_kP = 0.1;
 public static final double ELEVATOR_kI = 0.0;
 public static final double ELEVATOR_kD = 0.01;

 // Elevator Setpoints
 public static final double ELEVATOR_LOW = 0.0;  // Lowest position
 public static final double ELEVATOR_HIGH = 100.0; // Highest position
}

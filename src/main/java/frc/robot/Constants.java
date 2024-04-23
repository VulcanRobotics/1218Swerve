// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode currentMode = Mode.REAL;
  // public static final Mode currentMode = Mode.SIM;
  // public static final Mode currentMode = Mode.REPLAY;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Vision {
    public static final String kCameraName = "FrontCam";
    public static final String kCameraNameFL = "FrontLeftCam";
    public static final String kCameraNameFR = "FrontRightCam";
    public static final String kCameraNameBL = "BackLeftCam";
    public static final String kCameraNameBR = "BackRightCam";

    // private static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0);
    // private static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
    private static final double PI = Math.PI;

    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToCam =
        new Transform3d(
            // Camera faces back and titled up (yaw 180 and negative pitch from perspective of
            // robot).
            new Translation3d(0.089, -0.254, 0.3048), new Rotation3d(0, 0.349, 0));

    public static final Transform3d kRobotToCamFL =
        new Transform3d(
            // Camera faces back and titled up (yaw 180 and negative pitch from perspective of
            // robot).
            // new Translation3d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0, 0.3),
            new Translation3d(
                Units.inchesToMeters(10.5), Units.inchesToMeters(10.5), Units.inchesToMeters(8.0)),
            new Rotation3d(0, Math.toRadians(-16), PI / 4.0));

    public static final Transform3d kRobotToCamFR =
        new Transform3d(
            // Camera faces back and titled up (yaw 180 and negative pitch from perspective of
            // robot).
            new Translation3d(
                Units.inchesToMeters(10.5), -Units.inchesToMeters(10.5), Units.inchesToMeters(8.0)),
            new Rotation3d(0, Math.toRadians(-16), -PI / 4.0));

    public static final Transform3d kRobotToCamBL =
        new Transform3d(
            // Camera faces back and titled up (yaw 180 and negative pitch from perspective of
            // robot).
            new Translation3d(
                -Units.inchesToMeters(12.5),
                Units.inchesToMeters(12.5),
                Units.inchesToMeters(11.5)),
            new Rotation3d(0, Math.toRadians(-16), 3.0 * PI / 4.0));

    public static final Transform3d kRobotToCamBR =
        new Transform3d(
            // Camera faces back and titled up (yaw 180 and negative pitch from perspective of
            // robot).
            new Translation3d(
                -Units.inchesToMeters(12.5),
                -Units.inchesToMeters(12.5),
                Units.inchesToMeters(11.5)),
            new Rotation3d(0, Math.toRadians(-16), PI));

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(3.0, 3.0, 3.0);

    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.25, 0.25, 1.0);

    public static final Matrix<N3, N1> kOdometryStdDevs = VecBuilder.fill(0.05, 0.05, 0.05);

    // These values shouldn't really matter but I'd just keep them as is.
    public static final Matrix<N3, N1> kDefaultVisionStdDevs = VecBuilder.fill(0.05, 0.05, 0.05);

    public static final double kVisionAutoMultiplier = 4.0;
  }
}
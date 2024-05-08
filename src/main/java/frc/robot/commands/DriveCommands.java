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

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * This class contains many commands that will can control the drive train. They all follow a
 * similar structure, and can easily be modified or duplicated and edited to fit a specific purpose,
 * like aiming at a limelight target. Always be sure to tune the PID values for each of these
 * functions because that will determine how well they run.
 */
public class DriveCommands {

  // ALL OF THESE PID VALUES WILL LIKELY NEED TO BE CHANGED BASED ON THE ROBOT AND SITUATION.
  private static final double DEADBAND = 0.1;
  private static final double AnglePIDValues[] = {0.45, 0.0, 0.0};
  private static final double TranslationPIDValues[] = {1.2, 0.0, 0.01};
  private static final PIDController xTranslationController =
      new PIDController(TranslationPIDValues[0], TranslationPIDValues[1], TranslationPIDValues[2]);
  private static final PIDController yTranslationController =
      new PIDController(TranslationPIDValues[0], TranslationPIDValues[1], TranslationPIDValues[2]);
  private static final PIDController angleController =
      new PIDController(AnglePIDValues[0], AnglePIDValues[1], AnglePIDValues[2]);

  private DriveCommands() {}

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   *
   * @param fieldCentric Change this depending on whether the robot should drive field centric or
   *     not.
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      boolean fieldCentric) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          if (fieldCentric) {
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega * drive.getMaxAngularSpeedRadPerSec(),
                    isFlipped
                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                        : drive.getRotation()));

          } else {
            drive.runVelocity(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                    omega * drive.getMaxAngularSpeedRadPerSec(),
                    isFlipped
                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                        : drive.getRotation()));
          }
        },
        drive);
  }

  /**
   * This function returns a command that allows the driver to control the translation of the robot
   * while the drivetrain rotates to aim at a specified Rotation2d. This version uses a supplier so
   * that the taret can constantly be updated. The rotation ofth
   *
   * @param thetaSupplier This is the target to aim the drivebase at. If it's a constant value, use:
   *     "() -> value". If the value should update, use "class::function"
   */
  public static Command driveWhileAiming(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> thetaSupplier) {

    angleController.reset();
    angleController.setTolerance(Math.toRadians(0.25));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
        () -> {
          Pose2d robotPose = drive.getPose();
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          Rotation2d targetTheta = thetaSupplier.get();

          var omega =
              angleController.calculate(
                  robotPose.getRotation().getRadians(),
                  targetTheta.getRadians()
                      + (DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red
                          ? Math.PI
                          : 0.0));

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * This commands is extremely useful the there is any part of the field that you need to line up
   * with. For example, the charge nodes from 2023 Charged Up or the Amp from 2024 Crescendo. This
   * will automatically drive to a specified position in a linear path, given that the pose estimate
   * is accurate. It can take a lot of load of the driver to make hard alignments.
   *
   * @param drive Drive subsystem to pass.
   * @param poseSupplier Supplier for robot's current pose. Usually something like "drive::getPose"
   * @param targetPose The target pose to drive to. In supplier form so that it can be dynamic if
   *     necessary.
   * @return Commands that drives in a linear path to the specified position.
   */
  public static Command driveToPosition(
      Drive drive, Supplier<Pose2d> poseSupplier, Supplier<Pose2d> targetPose) {

    angleController.reset();
    angleController.setTolerance(Math.toRadians(0.25));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    yTranslationController.reset();
    xTranslationController.reset();
    yTranslationController.setTolerance(0.05);
    xTranslationController.setTolerance(0.05);
    yTranslationController.setSetpoint(targetPose.get().getTranslation().getY());
    xTranslationController.setSetpoint(targetPose.get().getTranslation().getX());

    return Commands.run(
        () -> {
          Pose2d pose = poseSupplier.get();
          double x = poseSupplier.get().getX();
          double y = poseSupplier.get().getY();
          double xSpeed = xTranslationController.calculate(x);
          double ySpeed = yTranslationController.calculate(y);

          // Change these limits for speed depending on what you plan to use this for.
          xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
          ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);

          double thetaSpeed =
              angleController.calculate(
                  pose.getRotation().getRadians(),
                  (-Math.PI / 2)
                      + (DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Red
                          ? Math.PI
                          : 0.0));

          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;

          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  xSpeed * drive.getMaxLinearSpeedMetersPerSec(),
                  ySpeed * drive.getMaxLinearSpeedMetersPerSec(),
                  thetaSpeed * drive.getMaxAngularSpeedRadPerSec(),
                  (isFlipped)
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * This commands is basically the same as regular drive to position, HOWEVER it has an END
   * condition, which means that the command scheduler knows when this command has been completeted
   * and then deschedules it. This is most useful in auto where commands must be descheduled before
   * the auto routine can continue. This same command is also available to see under
   * "DriveToPosition.java" where it is implemented as a command class instead of a function but
   * accomplishes the exact same thing.
   *
   * @param poseSupplier Supplier for robot's current pose. Usually something like "drive::getPose"
   * @param targetPose The target pose to drive to. In supplier form so that it can be dynamic if
   *     necessary.
   * @return Commands that drives in a linear path to the specified position.
   */
  public static Command driveToPositionThenStop(
      Drive drive, Supplier<Pose2d> poseSupplier, Supplier<Pose2d> targetPose) {

    angleController.reset();
    angleController.setTolerance(Math.toRadians(0.25));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    yTranslationController.reset();
    xTranslationController.reset();
    yTranslationController.setTolerance(0.05);
    xTranslationController.setTolerance(0.05);
    yTranslationController.setSetpoint(targetPose.get().getTranslation().getY());
    xTranslationController.setSetpoint(targetPose.get().getTranslation().getX());

    return Commands.run(
            () -> {
              Pose2d pose = poseSupplier.get();
              double x = poseSupplier.get().getX();
              double y = poseSupplier.get().getY();
              double xSpeed = xTranslationController.calculate(x);
              double ySpeed = yTranslationController.calculate(y);

              // Change these limits for speed depending on what you plan to use this for.
              xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
              ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);

              double thetaSpeed =
                  angleController.calculate(
                      pose.getRotation().getRadians(),
                      (-Math.PI / 2)
                          + (DriverStation.getAlliance().isPresent()
                                  && DriverStation.getAlliance().get() == Alliance.Red
                              ? Math.PI
                              : 0.0));

              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      xSpeed * drive.getMaxLinearSpeedMetersPerSec(),
                      ySpeed * drive.getMaxLinearSpeedMetersPerSec(),
                      thetaSpeed * drive.getMaxAngularSpeedRadPerSec(),
                      (isFlipped)
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)
        .onlyWhile(DriveCommands::atPosition);
  }

  private static boolean atPosition() {
    if (angleController.atSetpoint()
        && yTranslationController.atSetpoint()
        && xTranslationController.atSetpoint()) {
      return true;
    } else {
      return false;
    }
  }
}

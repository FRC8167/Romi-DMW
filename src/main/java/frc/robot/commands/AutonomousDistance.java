// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(Drivetrain drivetrain) {
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts, 
                                   DriveConstants.kvVoltSecondsPerMeter, 
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        10);

  TrajectoryConfig configForward =
    new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
                         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics)
        .addConstraint(autoVoltageConstraint)
        .setReversed(false);

        TrajectoryConfig configBackward =
        new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
                             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint)
            .setReversed(true);
        Trajectory trajectorypath1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(Units.inchesToMeters(15), Units.inchesToMeters(7.5)),
                new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(-7.5)),
                new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(7.5)),
                new Translation2d(Units.inchesToMeters(15), Units.inchesToMeters(-7.5))            
            ),
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(15), new Rotation2d(Math.PI)),
            configForward);

        Trajectory trajectorypath2 = TrajectoryGenerator.generateTrajectory(
              new Pose2d(0, 0, new Rotation2d(Math.PI)),
              List.of(
                  new Translation2d(Units.inchesToMeters(15), Units.inchesToMeters(7.5)),
                  new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(-7.5)),
                  new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(7.5)),
                  new Translation2d(Units.inchesToMeters(15), Units.inchesToMeters(-7.5))            
              ),
              new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(15), new Rotation2d(Math.PI)),
              configBackward);
    
    addCommands(
      new RamseteCommand(
        trajectorypath1,
        drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        drivetrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        drivetrain::tankDriveVolts,
        drivetrain),

      new RamseteCommand(
          trajectorypath2,
          drivetrain::getPose,
          new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
          DriveConstants.kDriveKinematics,
          drivetrain::getWheelSpeeds,
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          drivetrain::tankDriveVolts,
          drivetrain),

        new DriveDistance(-0.5, 10, drivetrain),
        new TurnDegrees(-0.5, 180, drivetrain),
        new DriveDistance(-0.5, 10, drivetrain),
        new TurnDegrees(0.5, 180, drivetrain));
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsytem;

/**
 * Add your docs here.
 */
public class SneakyTrajectory {
    public Trajectory FirstBlock_0 , FirstBlock_1;
    public Trajectory SecondBlck_0 , SecondBlock_1;
    public Trajectory ThirdBlock_0 , ThirdBlock_1;
    public Trajectory FourthBlock_0 , FourthBlock_1;
    public Trajectory FifthBlock_0 , FifthBlock_1;
    private DriveSubsytem m_drive;


    public SneakyTrajectory(DriveSubsytem drive){
                
        m_drive = drive;
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);
            TrajectoryConfig configForward =
   new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
     DriveConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(Constants.kDriveKinematics)
     // Apply the voltage constraint
     .addConstraint(autoVoltageConstraint);

     TrajectoryConfig configBackward =
   new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
     DriveConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(Constants.kDriveKinematics)
     // Apply the voltage constraint
     .addConstraint(autoVoltageConstraint);
     configBackward.setReversed(true);

     FirstBlock_0 = TrajectoryGenerator.generateTrajectory(
       List.of(
         new Pose2d(7.03 , 4.631, new Rotation2d(0)),
         new Pose2d(8.985 , 6.671, new Rotation2d(0))
       ), configForward);
      FirstBlock_1 = TrajectoryGenerator.generateTrajectory(
        List.of(
         new Pose2d(8.985 , 6.671, new Rotation2d(0)),
         new Pose2d(11.553 , 6.396, new Rotation2d(-51))
        ), configForward);
      SecondBlck_0 = TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(7.03 , 4.631, new Rotation2d(0)),
          new Pose2d(8.933 , 5.64, new Rotation2d(0))
        ), configForward);
      SecondBlock_1 = TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(8.933 , 5.64, new Rotation2d(0)),
          new Pose2d(11.15 , 5.232, new Rotation2d(-27))
        ), configForward);
      ThirdBlock_0 = TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(7.052 , 3.67, new Rotation2d(0)),
          new Pose2d(9.023 , 4.606, new Rotation2d(0))
        ), configForward);
      ThirdBlock_1 = TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(9.023 , 4.606, new Rotation2d(0)),
          new Pose2d(11.284 , 3.623, new Rotation2d(0))
        ), configForward);
      FourthBlock_0 = TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(7.052 , 3.67, new Rotation2d(0)),
          new Pose2d(8.989 , 3.622, new Rotation2d(0))
        ), configForward);
      FourthBlock_1 = TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(8.989 , 3.622, new Rotation2d(0)),
          new Pose2d(11.082 , 3.623, new Rotation2d(0))
        ), configForward);
      FifthBlock_0 = TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(7.198 , 2.517, new Rotation2d(0)),
          new Pose2d(8.91 , 2.566, new Rotation2d(0))
        ), configForward);
      FifthBlock_1 = TrajectoryGenerator.generateTrajectory(
        List.of(
          new Pose2d(8.91 , 2.566, new Rotation2d(0)),
          new Pose2d(11.172 , 2.686, new Rotation2d(13))
        ), configForward);
    }
    public RamseteCommand getRamsete(Trajectory trajectory ){

        return new RamseteCommand(
        trajectory,
        m_drive::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        m_drive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drive::tankDriveVolts,
        m_drive
    );
    }


}

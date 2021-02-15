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
import frc.robot.commands.Autonomous.PathATogether;
import frc.robot.subsystems.DriveSubsytem;

/**
 * Add your docs here.
 */
public class SneakyTrajectory {
  public Trajectory[] PathARed = new Trajectory[2];
  public Trajectory[] PathABlue = new Trajectory[2];
  public Trajectory[] PathATogether = new Trajectory[3];
    
    private DriveSubsytem m_drive;


    public SneakyTrajectory(DriveSubsytem drive){
                
        m_drive = drive;
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);
            TrajectoryConfig configForward =
   new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
     DriveConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics)
     // Apply the voltage constraint
     .addConstraint(autoVoltageConstraint);

     TrajectoryConfig configBackward =
   new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
     DriveConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics)
     // Apply the voltage constraint
     .addConstraint(autoVoltageConstraint);
     configBackward.setReversed(true);

     PathARed[0] = TrajectoryGenerator.generateTrajectory( 
     List.of(
         new Pose2d(0.74, 4.775, new Rotation2d(-2.9671)), 
         new Pose2d(4, 4, new Rotation2d(2.4435)),
         new Pose2d(6.57, 2.58, new Rotation2d(-2.1293))),
         configBackward);
      
         PathARed[1] = TrajectoryGenerator.generateTrajectory( 
         List.of(
             new Pose2d(6.57, 2.58, new Rotation2d(-2.1293)), 
             new Pose2d(15.3, 5.7, new Rotation2d(0))),
             configForward);

     PathABlue[0] = TrajectoryGenerator.generateTrajectory( 
     List.of(
         new Pose2d(0.75, 1.38, new Rotation2d(-3.1416)), 
         new Pose2d(8, 1.33, new Rotation2d(2.618)),
         new Pose2d(9.34, 5.43, new Rotation2d(-1.9199))),
         configBackward);
               
         PathABlue[1] = TrajectoryGenerator.generateTrajectory( 
         List.of(
             new Pose2d(9.34, 5.43, new Rotation2d(2.7925)), 
             new Pose2d(12, 4, new Rotation2d(2.618)),
             new Pose2d(15, 3, new Rotation2d(2.9671))),
             configForward);

     PathATogether[0] = TrajectoryGenerator.generateTrajectory( 
     List.of(
         new Pose2d(0.7, 5, new Rotation2d(-3.1416)), 
         new Pose2d(4, 4, new Rotation2d(2.618)),
         new Pose2d(6.7, 2.7, new Rotation2d(2.4435)),
         new Pose2d(8, 1.4, new Rotation2d(2.2689))),
         configBackward);
                        
         PathATogether[1] = TrajectoryGenerator.generateTrajectory( 
         List.of(
             new Pose2d(8, 1.4, new Rotation2d(2.2689)),
             new Pose2d(7, 6.8, new Rotation2d(-3.14169))),
             configForward);
             
             PathATogether[2] = TrajectoryGenerator.generateTrajectory( 
             List.of(
               new Pose2d(7, 6.8, new Rotation2d(-3.14169)),
               new Pose2d(8, 6.8, new Rotation2d(2.618)), 
               new Pose2d(9.3, 5.5, new Rotation2d(2.3213)),
               new Pose2d(12, 4, new Rotation2d(2.9671)),
               new Pose2d(15.2, 3.4, new Rotation2d(2.7925))),
               configForward);
        



    }
    public RamseteCommand getRamsete(Trajectory trajectory ){

        return new RamseteCommand(
        trajectory,
        m_drive::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_drive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drive::tankDriveVolts,
        m_drive
    );
    }


}

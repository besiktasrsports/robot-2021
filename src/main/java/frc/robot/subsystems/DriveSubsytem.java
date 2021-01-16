// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsytem extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private final WPI_VictorSPX leftRearMotor = new WPI_VictorSPX(DriveConstants.kLeftRearMotor);
  private final WPI_VictorSPX rightRearMotor = new WPI_VictorSPX(DriveConstants.kRightRearMotor);
  private final WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(DriveConstants.kLeftFrontMotor);
  private final WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(DriveConstants.kRightFrontMotor);
  private final DifferentialDrive m_drive = new DifferentialDrive(leftRearMotor, rightRearMotor);
  public DriveSubsytem() {
    leftFrontMotor.follow(leftRearMotor);
    rightFrontMotor.follow(rightRearMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftRearMotor.setVoltage(leftVolts);
    rightRearMotor.setVoltage(-rightVolts);
    m_drive.feed();
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot, true);
  }
}

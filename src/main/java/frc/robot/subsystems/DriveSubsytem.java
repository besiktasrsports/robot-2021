// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsytem extends SubsystemBase {
  /** Creates a new DriveTrain. */
  private final VictorSPX leftRearMotor = new VictorSPX(DriveConstants.lefRearMotor);
  private final TalonSRX leftFrontMotor = new TalonSRX(DriveConstants.leftFrontMotor);
  private final VictorSPX rightRearMotor = new VictorSPX(DriveConstants.rightRearMotor);
  private final TalonSRX rightFrontMotor = new TalonSRX(DriveConstants.rightFrontMotor);
  public DriveSubsytem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

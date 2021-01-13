// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hooper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoopperSubsystem;

public class HoopperCommand extends CommandBase {
  /** Creates a new HooperCommand. */
  private final HoopperSubsystem m_hoopper;
  private final double m_speed;
  public HoopperCommand(HoopperSubsystem hoopper, double speed) {
    m_speed = speed;
    m_hoopper = hoopper;
    addRequirements(m_hoopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_hoopper.runHooper(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hoopper.stopHopper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

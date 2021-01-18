// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.funnel.FunnelCommand;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.ToggleCompressor;
import frc.robot.commands.intake.ToggleDropIntake;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.drivetrain.JoystickDriveCommand;
import frc.robot.commands.Accelarator.AcceleratorCommand;
import frc.robot.commands.turret.TurretJoystickCommand;
import frc.robot.subsystems.AcceleratorSubsystem;
import frc.robot.subsystems.DriveSubsytem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public Joystick m_driverController = new Joystick(JoystickConstants.kDriverControllerPort);
  public Joystick m_operatorController = new Joystick(JoystickConstants.kOperatorControllerPort);
  public final FunnelSubsystem m_funnel = new FunnelSubsystem();
  public final TurretSubsystem m_turret = new TurretSubsystem();
  public final AcceleratorSubsystem m_accelerator = new AcceleratorSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public final DriveSubsytem m_robotDrive = new DriveSubsytem();
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_robotDrive.setDefaultCommand(new JoystickDriveCommand(m_robotDrive, () -> -m_driverController.getRawAxis(1),
        () -> m_driverController.getRawAxis(0)));
  }

  private void configureButtonBindings() {

    // Turret Commands

    new JoystickButton(m_driverController, 1).whileHeld(new TurretJoystickCommand(m_turret, 0.3));
    new JoystickButton(m_driverController, 2).whileHeld(new FunnelCommand(m_funnel,0.5, 0.3));

    new JoystickButton(m_driverController, 1).whileHeld(new TurretJoystickCommand(m_turret, 0.3));

    // Intake Commands
    new JoystickButton(m_driverController, 2).whileHeld(new RunIntake(m_intake, 0.5));
    new JoystickButton(m_operatorController, 1).whileHeld(new ToggleDropIntake(m_intake));
    new JoystickButton(m_operatorController, 3).whileHeld(new ToggleCompressor(m_intake));
    
    // Shooter Commands
    new JoystickButton(m_driverController, 3).whileHeld(new RunShooter(m_shooter, 0.75));

    

  }

    new JoystickButton(m_driverController, 4).whileHeld(new AcceleratorCommand(m_accelerator, 0.3));
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}

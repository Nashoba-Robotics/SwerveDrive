// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ConfigOffsetCommand;
import frc.robot.commands.ContinuityTestCommand;
import frc.robot.commands.JoystickTurnCommand;
import frc.robot.commands.SingleSwerveDriveCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwerveTestCommand;
import frc.robot.commands.ToZeroCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putData(new SwerveTestCommand());
    SmartDashboard.putData(new ToZeroCommand());
    SmartDashboard.putData(new ConfigOffsetCommand());
    SmartDashboard.putData(new JoystickTurnCommand());
    SmartDashboard.putData(new SwerveDriveCommand());
    SmartDashboard.putData(new ContinuityTestCommand());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}
}

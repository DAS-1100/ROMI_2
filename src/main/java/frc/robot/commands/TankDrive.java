// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class TankDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_yaxisSpeedSupplier;  
  
 /**
   * Creates a new TankDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xaxisSpeedSupplier Lambda supplier of left speed
   * @param yaxisSpeedSupplier Lambda supplier of right speed
   */
  
  public TankDrive(
    Drivetrain drivetrain,
    Supplier<Double> xaxisSpeedSupplier,
    Supplier<Double> yaxisSpeedSupplier) {
  m_drivetrain = drivetrain;
  m_xaxisSpeedSupplier = xaxisSpeedSupplier;
  m_yaxisSpeedSupplier = yaxisSpeedSupplier;
  addRequirements(drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  public  void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  public  void execute() {
     m_drivetrain.tankDrive(m_xaxisSpeedSupplier.get(), m_yaxisSpeedSupplier.get());
  }


  // Called once after isFinished returns true
  @Override
  public  void end(boolean interrupted) {}
 // Make this return true when this Command no longer needs to run execute()

 @Override
 public  boolean isFinished() {
   return false;
 }
}

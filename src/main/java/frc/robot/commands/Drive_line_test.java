// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Drivetrain;

public class Drive_line_test extends CommandBase {
  /** Creates a new Diive_line_test. */
  private DigitalInput m_input_DVal0 = new DigitalInput(8);
  private DigitalInput m_input_DVal1 = new DigitalInput(9);
  private DigitalInput m_input_DVal2 = new DigitalInput(10);
  private DigitalInput m_input_DVal3 = new DigitalInput(11);
  private DigitalInput m_input_DVal4 = new DigitalInput(12);

  private final Drivetrain m_drive;
  private final double m_speed_f;
  private final double m_speed_t;
  private final double m_speed_s;

  private double  Sensor_val;
  private double  Sensor_val0;
  private double  Sensor_val1;
  private double  Sensor_val2;
  private double  Sensor_val3;
  private double  Sensor_val4;

  private double left_speed;
  private double right_speed;
  private boolean stop_drive;

  public Drive_line_test(double speed_f, double speed_t, double speed_s, Drivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_speed_f = speed_f;
    m_speed_t = speed_t;
    m_speed_s = speed_s;
    m_drive = drive;
    addRequirements(drive); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
// Calibrate Sensors
m_drive.arcadeDrive(0, 0);
m_drive.resetEncoders();
stop_drive = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_input_DVal0.get()) {Sensor_val0 = 1;}
    if (m_input_DVal1.get()) {Sensor_val1 = 2;}
    if (m_input_DVal2.get()) {Sensor_val2 = 4;}
    if (m_input_DVal3.get()) {Sensor_val3 = 8;}
    if (m_input_DVal4.get()) {Sensor_val4 = 16;}

    Sensor_val = (Sensor_val0 + Sensor_val1 + Sensor_val2 + Sensor_val3 + Sensor_val4);
    
    SmartDashboard.putBoolean("DIO 0", m_input_DVal0.get());
    SmartDashboard.putBoolean("DIO 1", m_input_DVal1.get());
    SmartDashboard.putBoolean("DIO 2", m_input_DVal2.get());
    SmartDashboard.putBoolean("DIO 3", m_input_DVal3.get());
    SmartDashboard.putBoolean("DIO 4", m_input_DVal4.get());
    SmartDashboard.putNumber("Sensor_Val", Sensor_val);

    Sensor_val0 = 0;
    Sensor_val1 = 0;
    Sensor_val2 = 0;
    Sensor_val3 = 0;
    Sensor_val4 = 0;



// Move forward straight

  if (Sensor_val == 4){
    left_speed = ( m_speed_f );
    right_speed = ( m_speed_f + m_speed_s );
  }
 if (Sensor_val == 2){
  left_speed = ( m_speed_f );
  right_speed = ( m_speed_f - m_speed_t );
    }
    if (Sensor_val == 8){
      left_speed = ( m_speed_f - m_speed_t );
      right_speed = ( m_speed_f );
        }
        if (Sensor_val == 1){
          left_speed = ( m_speed_f + m_speed_t );
          right_speed = ( m_speed_f - m_speed_t );
            }
            if (Sensor_val == 16){
              left_speed = ( m_speed_f - m_speed_t );
              right_speed = ( m_speed_f + m_speed_t );
                }
              
                if (Sensor_val == 31){
                  left_speed = ( 0 );
                  right_speed = ( 0 );
                  stop_drive = true;
                    }
                  

SmartDashboard.putNumber("left_speed", left_speed);
SmartDashboard.putNumber("right_speed", right_speed);

m_drive.tankDrive(left_speed, right_speed);




}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop_drive;
  }

  //private double getAverageTurningDistance() {
   // double leftDistance = Math.abs(m_drive.getLeftDistanceInch());
   // double rightDistance = Math.abs(m_drive.getRightDistanceInch());
   // return (leftDistance + rightDistance) / 2.0;
 // }


}

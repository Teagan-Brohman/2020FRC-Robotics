/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SensorType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogGyro;



public class Robot extends TimedRobot {
  private MecanumDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private static final int leftFrontDeviceID = 1; 
  private static final int leftBackDeviceID = 2;
  private static final int rightFrontDeviceID = 3;
  private static final int rightBackDeviceID = 4;
  private CANSparkMax m_leftFrontMotor;
  private CANSparkMax m_leftBackMotor;
  private CANSparkMax m_rightFrontMotor;
  private CANSparkMax m_rightBackMotor;
  private CANEncoder m_leftBackEncoder;
  private CANEncoder m_leftFrontEncoder;
  private CANEncoder m_rightBackEncoder;
  private CANEncoder m_rightFrontEncoder;
  private static final int kGyroPort = 0;

  private final AnalogGyro m_gyro = new AnalogGyro(kGyroPort);

  public double yValue;
  public double xValue;
  public float leftPower;
  public float rightPower;

  @Override
  public void robotInit() {
  /**
   * SPARK MAX controllers are intialized over CAN by constructing a CANSparkMax object
   * 
   * The CAN ID, which can be configured using the SPARK MAX Client, is passed as the
   * first parameter
   * 
   * The motor type is passed as the second parameter. Motor type can either be:
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless
   *  com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushed
   * 
   * The example below initializes four brushless motors with CAN IDs 1 and 2. Change
   * these parameters to match your setup
   */
    m_leftFrontMotor = new CANSparkMax(leftFrontDeviceID, MotorType.kBrushless);
    m_leftBackMotor = new CANSparkMax(leftBackDeviceID, MotorType.kBrushless);
    m_rightFrontMotor = new CANSparkMax(rightFrontDeviceID, MotorType.kBrushless);
    m_rightBackMotor = new CANSparkMax(rightBackDeviceID, MotorType.kBrushless);
    
    m_leftFrontEncoder = m_leftFrontMotor.getEncoder();
    m_leftBackEncoder = m_leftBackMotor.getEncoder();
    m_rightFrontEncoder = m_rightFrontMotor.getEncoder();
    m_rightBackEncoder = m_rightBackMotor.getEncoder();
    
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    // m_leftMotor.restoreFactoryDefaults();
    // m_rightMotor.restoreFactoryDefaults();

    
      m_myRobot = new MecanumDrive(m_leftFrontMotor, m_leftBackMotor, m_rightFrontMotor, m_rightBackMotor);
   // m_myRobot = new DifferentialDrive(m_leftMotor, m_rightMotor);

    m_leftStick = new Joystick(0);
    //m_rightStick = new Joystick(0);
    
  }

  @Override
  public void teleopPeriodic() {

    yValue = m_leftStick.getY();
    xValue = m_leftStick.getX();



    m_myRobot.driveCartesian(yValue, xValue, m_leftStick.getZ(), m_gyro.getAngle());


    //m_myRobot.tankDrive(m_leftStick.getX(), m_leftStick.getZ());
    // m_leftFrontMotor.set(m_leftStick.getY());
    // m_leftBackMotor.set(m_leftStick.getY());
    // m_rightFrontMotor.set(m_leftStick.getZ());
    // m_rightBackMotor.set(m_leftStick.getZ());
    //System.out.println(m_leftStick.getX());
    //System.out.println(m_rightEncoder.getPosition());
    
  }
}

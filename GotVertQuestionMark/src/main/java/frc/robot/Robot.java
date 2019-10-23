/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SensorType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SolenoidBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.networktables.*;

import frc.robot.subsystems.SmartDash;




public class Robot extends TimedRobot {

  // private AHRS ahrs; 
  AHRS ahrs;
  private MecanumDrive m_myRobot;
  private Joystick m_leftStick;
  public static final int kGamepadButtonA = 1;
  private static final int leftFrontDeviceID = 1; 
  private static final int leftBackDeviceID = 3;
  private static final int rightFrontDeviceID = 5;
  private static final int rightBackDeviceID = 2;
  private CANSparkMax m_leftFrontMotor;
  private CANSparkMax m_leftBackMotor;
  private CANSparkMax m_rightFrontMotor;
  private CANSparkMax m_rightBackMotor;
  private CANEncoder m_leftBackEncoder;
  private CANEncoder m_leftFrontEncoder;                                                                                 
  private CANEncoder m_rightBackEncoder;
  private CANEncoder m_rightFrontEncoder;
  
  public double yValue;
  public double xValue;
  public double zValue;
  public float leftPower;
  public float rightPower;
  public double roboGyro; 

  private boolean m_LimelightHasValidTarget = false;
  private double yPower = 0.0;
  private double xPower = 0.0;
  
  Compressor m_compressor = new Compressor(0);
  DoubleSolenoid solenoidDouble = new DoubleSolenoid(1, 2);

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
    
    m_myRobot = new MecanumDrive(m_leftFrontMotor, m_leftBackMotor, m_rightFrontMotor, m_rightBackMotor);

    m_leftBackEncoder = m_leftFrontMotor.getEncoder();
    m_leftBackEncoder = m_leftBackMotor.getEncoder();
    m_rightFrontEncoder = m_rightFrontMotor.getEncoder();
    m_rightBackEncoder = m_rightBackMotor.getEncoder();
    
    m_leftFrontMotor.setIdleMode(IdleMode.kBrake);
    m_rightFrontMotor.setIdleMode(IdleMode.kBrake);
    m_leftBackMotor.setIdleMode(IdleMode.kBrake);
    m_rightBackMotor.setIdleMode(IdleMode.kBrake);
 
    m_compressor.setClosedLoopControl(true);

    
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    // m_leftMotor.restoreFactoryDefaults();
    // m_rightMotor.restoreFactoryDefaults();

    m_leftStick = new Joystick(1); 
    //m_rightStick = new Joystick(0);

    try {
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex ) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
    }
  }

  @Override
  public void teleopPeriodic() {

    yValue = m_leftStick.getY();
    xValue = m_leftStick.getX();
    zValue = m_leftStick.getZ();
    
    xValue = xValue/2;
    yValue = yValue/2;
    zValue = zValue/4;

    if(xValue < 0.05 && xValue > -0.05){
      xValue = 0;
    }
    if(yValue < 0.05 && yValue > -0.05){
      yValue = 0;
    }
    if(zValue < 0.05 && zValue > -0.05){
      zValue = 0;
    }

    roboGyro = ahrs.getAngle();

    if(m_leftStick.getRawButton(1)){ //trigger pressed
      Update_Limelight_Tracking();
      m_myRobot.driveCartesian(-xPower, -yPower, 0, -roboGyro);
    }
    else{//Trigger not pressed = normal drive
      m_myRobot.driveCartesian(-xValue, yValue, -zValue, -roboGyro);
      Update_Limelight_Tracking();
    }
    
    if(m_leftStick.getRawButton(11)){
      solenoidDouble.set(DoubleSolenoid.Value.kForward);
    }
    else if(m_leftStick.getRawButton(12)){
      solenoidDouble.set(DoubleSolenoid.Value.kReverse);
    } 
    else{
      solenoidDouble.set(DoubleSolenoid.Value.kOff);
    }
    
  
 if (m_leftStick.getRawButton(3)){
                 ahrs.reset();
                 
             }

    SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
    SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
    SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
    SmartDashboard.putNumber(   "Angle",             ahrs.getAngle());
    SmartDashboard.putBoolean("Compressor Enabled? ", m_compressor.enabled());
    SmartDashboard.putBoolean("Pressure Switch Open? ", m_compressor.getPressureSwitchValue());
    SmartDashboard.putNumber("Compressor Current Value: ", m_compressor.getCompressorCurrent());

    SmartDashboard.updateValues();
  }
  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double xCoef = 0.015;                    // how hard to turn toward the target
        final double yCoef = 0.025;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.2;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          yPower = 0.0;
          xPower = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double x = tx * xCoef; 
        if(x > MAX_DRIVE)
        {
          x = MAX_DRIVE;
        }
        xPower = x;

        // try to drive forward until the target area reaches our desired area
        double y = (DESIRED_TARGET_AREA - ta) * yCoef;
        if (y > MAX_DRIVE) // don't let the robot drive too fast into the goal
        {
          y = MAX_DRIVE;
        }
        yPower = y;
        SmartDashboard.putNumber("ta", ta);

  }
}
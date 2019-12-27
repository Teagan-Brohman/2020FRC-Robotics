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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.video.Video;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SolenoidBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.*;

import edu.wpi.first.wpilibj.Encoder;

import frc.robot.subsystems.SmartDash;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PWMSpeedController;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import com.revrobotics.ControlType;
import com.revrobotics.CANPIDController;

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
  private static final int turretDeviceID = 6;
  private static final int kServoID = 7;
  private static double turretPower = 0.0;
  private CANSparkMax m_leftFrontMotor;
  private CANSparkMax m_leftBackMotor;
  private CANSparkMax m_rightFrontMotor;
  private CANSparkMax m_rightBackMotor;
  private CANSparkMax m_turretMotor;
  private CANEncoder m_turretEncoder;
  private CANPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private CANEncoder m_leftBackEncoder;
  private CANEncoder m_leftFrontEncoder;                                                                                 
  private CANEncoder m_rightBackEncoder;
  private CANEncoder m_rightFrontEncoder;
  private Servo brahServo;
 
 
  
  public double yValue;
  public double xValue;
  public double zValue;
  public float leftPower;
  public float rightPower;
  public double roboGyro; 
  
  Compressor m_compressor = new Compressor(0);
  DoubleSolenoid solenoidDouble = new DoubleSolenoid(1, 2);


  DigitalInput limitSwitch = new DigitalInput(1);
  AnalogPotentiometer pot = new AnalogPotentiometer(1);
  // CANTalon m_turretEncoder = new CANTalon();

  public UsbCamera drive;
  public VideoMode videoMode;

  private double servoDegree;

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
    
    m_turretMotor = new CANSparkMax(turretDeviceID, MotorType.kBrushed);

   
    // m_pidController = m_turretMotor.getPIDController();
    m_turretEncoder= m_turretMotor.getEncoder(SensorType.kEncoder, 1440);


    // kP = 0.1; 
    // kI = 1e-4;
    // kD = 1; 
    // kIz = 0; 
    // kFF = 0; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;

    // m_pidController.setP(kP);
    // m_pidController.setI(kI);
    // m_pidController.setD(kD);
    // m_pidController.setIZone(kIz);
    // m_pidController.setFF(kFF);
    // m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    brahServo = new Servo(kServoID);
    
    m_myRobot = new MecanumDrive(m_leftFrontMotor, m_leftBackMotor, m_rightFrontMotor, m_rightBackMotor);

    m_leftFrontEncoder = m_leftFrontMotor.getEncoder();
    m_leftBackEncoder = m_leftBackMotor.getEncoder();
    m_rightFrontEncoder = m_rightFrontMotor.getEncoder();
    m_rightBackEncoder = m_rightBackMotor.getEncoder();
    
    

    m_compressor.setClosedLoopControl(true);

    drive = CameraServer.getInstance().startAutomaticCapture();
    videoMode = new VideoMode(PixelFormat.kYUYV, 800, 448, 30);
    

    if(limitSwitch.get()){
      System.out.println("ya dumb");
    }
    
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

    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);
    // double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP)) { m_pidController.setP(p); kP = p; }
    // if((i != kI)) { m_pidController.setI(i); kI = i; }
    // if((d != kD)) { m_pidController.setD(d); kD = d; }
    // if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    // if((max != kMaxOutput) || (min != kMinOutput)) { 
    //   m_pidController.setOutputRange(min, max); 
    //   kMinOutput = min; kMaxOutput = max; 

    //   m_pidController.setReference(rotations, ControlType.kPosition);

    //   SmartDashboard.putNumber("SetPoint", rotations);
      SmartDashboard.putNumber("ProcessVariable", m_turretEncoder.getPosition());
      SmartDashboard.putNumber("CPR", m_turretEncoder.getCPR());
      

    yValue = m_leftStick.getY();
    xValue = m_leftStick.getX();
    zValue = m_leftStick.getZ();
    
    xValue = xValue;
    yValue = yValue;
    zValue = zValue/3;

    if(xValue < 0.05 && xValue > -0.05){
      xValue = 0;
    }
    if(yValue < 0.05 && yValue > -0.05){
      yValue = 0;
    }
    if(zValue < 0.05 && zValue > -0.05){
      zValue = 0;
    }
    
    if(m_leftStick.getRawButton(10)){
      m_turretMotor.set(0.5);
    }
    else if(m_leftStick.getRawButton(9)){
      m_turretMotor.set(-0.5);
    }
    else{
      m_turretMotor.set(0);
    }

    roboGyro = ahrs.getAngle();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx"); 
    double x = tx.getDouble(0.0);
    SmartDashboard.putNumber("LimelightX",  x);

    drive.setFPS(30);
    drive.setVideoMode(videoMode);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(2);

    if(m_leftStick.getRawButton(1)){ //trigger pressed
      if(x > 0){
        m_myRobot.driveCartesian(-(Math.pow((0.025 * x), 2)), yValue, 0, -roboGyro);
      }
      else if(x < 0){
        m_myRobot.driveCartesian((Math.pow((0.025 * x), 2)), yValue, 0, -roboGyro);
      }
      else{
        m_myRobot.driveCartesian(0, 0, 0, 0);
      }
    }
    else{//Trigger not pressed = normal drive
      m_myRobot.driveCartesian(xValue, -yValue, zValue, -roboGyro);

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
  if(m_leftStick.getRawButton(13)){
     //m_turretEncoder.
  }


if(m_leftStick.getRawButton(5)){
  
  
  // if(m_turretEncoder.getPosition() > 0){
  //   turretPower = 0.25;
  // } else if(m_turretEncoder.getPosition() < 0){
  //   turretPower = -0.25;
  // } 
}
//else{
//   if(x > 4){
//     turretPower += Math.abs(x/150);
//   } else if(x < -4){
//     turretPower -= Math.abs(x/150);
//   } else{
//     turretPower = 0;
//   }
// }
  
//   m_turretMotor.set(ControlMode.PercentOutput, turretPower);

  // //Spinning the turret and makeing a "reset Button"
  // if(m_leftStick.getRawButton(9)){
  //   brahServo.setAngle(90);
  // }
  // servoDegree = brahServo.getAngle();
  // if(x > 4){
  //   servoDegree += Math.abs(x/10);//.8
  //   brahServo.setAngle(servoDegree);
  // }
  // if(x < -4){
  //   servoDegree -= Math.abs(x/10);
  //   brahServo.setAngle(servoDegree);
  // }
  //SmartDashboard.putNumber("ServoDegree",  servoDegree);
  
  // if(m_leftStick.getRawButton(5)){
  //   brahServo.setAngle(180);
  // }             
  // if(m_leftStick.getRawButton(6)){
  //   brahServo.setAngle(0);
  // }
    // try {
    // /* Use the joystick X axis for lateral movement,            */
    // /* Y axis for forward movement, and Z axis for rotation.    */
    // /* Use navX MXP yaw angle to define Field-centric transform */
    //m_myRobot.setRightSideInverted(false);
    // //m_myRobot.driveCartesian(yValue, xValue, m_leftStick.getZ(), 0);
    // //m_myRobot.driveCartesian(yValue, xValue, m_leftStick.getZ(), ahrs.getAngle());
    // } catch( RuntimeException ex ) {
    // DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
    // }
  
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
    SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
    SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
    SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
    SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
    SmartDashboard.putNumber(   "Angle",             ahrs.getAngle());

     SmartDashboard.putNumber("Front Left", m_leftFrontEncoder.getPosition());
     SmartDashboard.putNumber("Front Right", m_rightFrontEncoder.getPosition());
     SmartDashboard.putNumber("Back Left", m_leftBackEncoder.getPosition());
     SmartDashboard.putNumber("Back Right", m_rightBackEncoder.getPosition());
     //SmartDashboard.putNumber("Turret", m_turretEncoder.getRaw());

    // SmartDashboard.putNumber("Front Right Power", m_rightFrontEncoder.getVelocity());
    // SmartDashboard.putNumber("Back Right Power", m_rightBackEncoder.getVelocity());
    // // SmartDashboard.putNumber("Front Left Power", m_leftFrontEncoder.getVelocity());
    // // SmartDashboard.putNumber("back Left Power", m_leftBackEncoder.getVelocity());
    //SmartDashboard.putBoolean("Are motors reversed", m_myRobot.isRightSideInverted());

    SmartDashboard.putBoolean("Compressor Enabled? ", m_compressor.enabled());
    SmartDashboard.putBoolean("Pressure Switch Open? ", m_compressor.getPressureSwitchValue());
    SmartDashboard.putNumber("Compressor Current Value: ", m_compressor.getCompressorCurrent());
   
    //SmartDashboard.putNumber("Double Solanoid Value ", DoubleSolenoid.Value.kReverse);
    
    SmartDashboard.updateValues();
  
    //m_myRobot.tankDrive(m_leftStick.getX(), m_leftStick.getZ());
     //m_leftFrontMotor.set(m_leftStick.getY());
    // m_leftBackMotor.set(m_leftStick.getY());
    // m_rightFrontMotor.set(m_leftStick.getZ());
    // m_rightBackMotor.set(m_leftStick.getZ());
    //System.out.println(m_leftStick.getX());
    //System.out.println(m_rightEncoder.getPosition());
  }
}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.studica.frc.AHRS;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final SparkMax leftmotor1 = new SparkMax(5, MotorType.kBrushed);//front
  private final SparkMax leftmotor2 = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax rightmotor1 = new SparkMax(4, MotorType.kBrushed);//front 
  private final SparkMax rightmotor2 = new SparkMax(2, MotorType.kBrushed);
  
  private final SparkMaxConfig leftMotor1 = new SparkMaxConfig();//frente
  private final SparkMaxConfig leftMotor2 = new SparkMaxConfig();
  private final SparkMaxConfig rightMotor1 = new SparkMaxConfig();//frente
  private final SparkMaxConfig rightMotor2 = new SparkMaxConfig();


  private final PS4Controller controller = new PS4Controller(0);

  private AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

  MecanumDrive mecanumDrive = new MecanumDrive(leftmotor1, leftmotor2, rightmotor1, rightmotor2);

  boolean fod;        //Habilitar o deshabilitar el control Field Oriented Drive.
  //private AHRS navx = new AHRS(AHRS.NavXComType.kUSB)
  


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choserr", m_chooser);



    leftMotor1.inverted(false).idleMode(IdleMode.kBrake);// frente
    leftMotor2.inverted(false).idleMode(IdleMode.kBrake);
    rightMotor1.inverted(true).idleMode(IdleMode.kBrake);//frente
    rightMotor2.inverted(true).idleMode(IdleMode.kBrake);

    leftmotor1.configure(leftMotor1, null, null);
    leftmotor2.configure(leftMotor2, null, null);
    rightmotor1.configure(rightMotor1, null, null);
    rightmotor2.configure(rightMotor2, null, null);




    



  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:}
        // Put default auto code here    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    navx.reset();
    SmartDashboard.putBoolean("FOD", fod);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double ySpeed = controller.getLeftY(); // Remember, this is reversed!
    double xSpeed = -controller.getLeftX(); // Counteract imperfect strafing
    double zRotation = -controller.getRightX();

    fod = SmartDashboard.getBoolean("FOD", fod);

     

    SmartDashboard.putNumber("Navx Angle", navx.getAngle());

    if (fod) {
         
      //POV
       
        mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation,Rotation2d.fromDegrees(navx.getAngle())); //manejo con navx FOD
       
    } else {

      mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation); //manejo sin navx NO FOD
    }

    

    if (controller.getOptionsButton() == true){
      navx.reset();
    }


  } 

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

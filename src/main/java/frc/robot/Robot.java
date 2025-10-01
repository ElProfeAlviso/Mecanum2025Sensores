package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import java.util.List;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default Auto";
  private static final String kCustomAuto = "My Auto Code";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final SparkMax frontLeftMotor = new SparkMax(5, MotorType.kBrushed);
  private final SparkMax rearLeftMotor = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax frontRightMotor = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax rearRightMotor = new SparkMax(2, MotorType.kBrushed);

  private final SparkMaxConfig frontLeftMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig rearLeftMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig frontRightMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig rearRightMotorConfig = new SparkMaxConfig();

  private final PS4Controller driverController = new PS4Controller(0);

  // private AHRS navx = new AHRS(AHRS.NavXComType.kUSB)
  private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private final MecanumDrive mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor,
      rearRightMotor);

  private boolean fod; // Habilitar o deshabilitar el control Field Oriented Drive.

  Timer cronos = new Timer(); // Nuevo timer para cronometrar tiempo de autonomo.;

  PowerDistribution PowerDistribution = new PowerDistribution(1, ModuleType.kCTRE);

  private final Field2d m_field = new Field2d();

  Alert alert = new Alert("Modo FOD ACTIVADO", Alert.AlertType.kInfo);  
  Alert alert2 = new Alert("PARO DE EMERGENCIA ACTIVADO", Alert.AlertType.kError);


  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public void robotInit() {

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choserr", m_chooser);

    frontLeftMotorConfig.inverted(true).idleMode(IdleMode.kBrake);
    rearLeftMotorConfig.inverted(true).idleMode(IdleMode.kBrake);
    frontRightMotorConfig.inverted(false).idleMode(IdleMode.kBrake);
    rearRightMotorConfig.inverted(false).idleMode(IdleMode.kBrake);

    frontLeftMotor.configure(frontLeftMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    rearLeftMotor.configure(rearLeftMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    frontRightMotor.configure(frontRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    rearRightMotor.configure(rearRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    mecanumDrive.setDeadband(0.02);

    Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

        SmartDashboard.putData(m_field);

        m_field.getObject("traj").setTrajectory(m_trajectory);




  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    cronos.start();
    cronos.reset();
    navx.reset();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:

        if (cronos.get() <= 3) {

          mecanumDrive.driveCartesian(0.5, 0, 0);

        } else {

          mecanumDrive.driveCartesian(0, 0, 0);

        }
      default:
    }
    // Put default auto code here }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    navx.reset();
    fod = true;
    SmartDashboard.putBoolean("FOD", fod);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double ySpeed = -driverController.getLeftY(); // Remember, this is reversed!
    double xSpeed = driverController.getLeftX(); // Counteract imperfect strafing
    double zRotation = driverController.getRightX();

    double matchTime = DriverStation.getMatchTime();
    

    fod = SmartDashboard.getBoolean("FOD", fod);

    SmartDashboard.putData("Navx Angle", navx);
    SmartDashboard.putData("Chasis", mecanumDrive);
    SmartDashboard.putData("PDP", PowerDistribution);
    SmartDashboard.putNumber("Match Time",matchTime);
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putBoolean("FOD", fod);
    alert.set(fod);
    alert2.set(DriverStation.isEStopped());

    if (fod) {

      Rotation2d navXAngle = Rotation2d.fromDegrees(navx.getAngle());

      // POV

      mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation, navXAngle);
     
                                               // navx FOD

    } else {

      mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
      
    }

    if (driverController.getOptionsButton() == true) {
      navx.reset();
    }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}

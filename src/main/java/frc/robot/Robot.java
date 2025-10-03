package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.Elastic;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Servo;

import java.util.List;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.LinearFilter;
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
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.epilogue.Logged;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;


/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 * 
 */

public class Robot extends TimedRobot {

  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);


    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private int rainbowFirstPixelHue = 0;


  private static final String kDefaultAuto = "Defaul Auto";
  private static final String kCustomAuto = "Line Auto";

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

  private Servo intakeServo = new Servo(0); // Servo para el mecanismo de intake.

  DigitalInput magneticSensor = new DigitalInput(4); // Sensor magnético en el eje X del robot.
  DigitalInput limitSwitch = new DigitalInput(6); // Sensor de limite
  BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.INFO, "Teleoperado iniciado", "El modo teleoperado inicio correctamente");


  AnalogPotentiometer Ultrasonic = new AnalogPotentiometer(0, 5000, 300);

   Encoder encoder4x = new Encoder(0, 1, true, Encoder.EncodingType.k4X);

   DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(2, 360, 0);


  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public void robotInit() {

        led = new AddressableLED(6);

        // Crea buffer de 6 LEDs
        ledBuffer = new AddressableLEDBuffer(5);
        led.setLength(ledBuffer.getLength());

        // Asigna buffer al objeto LED
        led.setData(ledBuffer);

        // Activa la señal
        led.start();

        for (int i = 0; i < ledBuffer.getLength(); i++) {
          ledBuffer.setRGB(i, 0, 0, 0); // R, G, B
      }
      led.setData(ledBuffer);

        

    enableLiveWindowInTest(true);

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
    mecanumDrive.setMaxOutput(1.0);
    boolean safetyDrive = mecanumDrive.isSafetyEnabled();

    Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

        SmartDashboard.putData(m_field);

        m_field.getObject("traj").setTrajectory(m_trajectory);

        intakeServo.setAngle(90); // Posición inicial del servo.

        // Inicia la captura automática de la primera cámara USB encontrada
    UsbCamera camera = CameraServer.startAutomaticCapture();

    // Configurar resolución y FPS (opcional, ajusta según rendimiento)
    camera.setResolution(320, 240);
    camera.setFPS(15);




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

    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    int proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putString("Color Sensor", m_colorSensor.getColor().toHexString());
   



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
      case kDefaultAuto:
        // Put custom auto code here
        break;
      case kCustomAuto:

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
    SmartDashboard.putNumber("Servo Angle", 90);
    SmartDashboard.putData("Chasis", mecanumDrive);
    SmartDashboard.putData("PDP", PowerDistribution);

    Elastic.sendNotification(notification);

    encoder4x.setSamplesToAverage(5);
    encoder4x.setDistancePerPulse(1.0/360*Math.PI*6);
    encoder4x.setMinRate(1);
    encoder4x.reset();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("Temperatura PDP", PowerDistribution.getTemperature());

    double ySpeed = -driverController.getLeftY(); // Remember, this is reversed!
    double xSpeed = driverController.getLeftX(); // Counteract imperfect strafing
    double zRotation = driverController.getRightX();

    double matchTime = DriverStation.getMatchTime();
    

    fod = SmartDashboard.getBoolean("FOD", fod);

    SmartDashboard.putData("Navx Angle", navx);

    LinearFilter xAccFilter = LinearFilter.movingAverage(10);
    SmartDashboard.putNumber("Accelerometro",xAccFilter.calculate(navx.getWorldLinearAccelX()));
    
    
    SmartDashboard.putNumber("Match Time",matchTime);
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putBoolean("FOD", fod);
    alert.setText("Modo Fiel Oriented Drive ACTIVADO");
    alert.set(fod);
    alert2.set(DriverStation.isEStopped());

    

    SmartDashboard.putBoolean("Magnetic Sensor", magneticSensor.get());
    SmartDashboard.putNumber("Ultrasonico",Ultrasonic.get());

    SmartDashboard.putData("Rio Acelerometro",accelerometer);

    double servoIncrement = SmartDashboard.getNumber("Servo Angle", 90);

    intakeServo.setAngle(servoIncrement); 

    SmartDashboard.putNumber("Encoder en Distancia", Math.round(encoder4x.getDistance() * 100) / 100d);
    SmartDashboard.putData("Encoder Relativo", encoder4x);

    
    SmartDashboard.putData("Encoder Absoluto", absoluteEncoder);
    SmartDashboard.putBoolean("Limit switch", limitSwitch.get());

    for (int i = 0; i < ledBuffer.getLength(); i++) {
      final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
  }
  rainbowFirstPixelHue += 3;
  rainbowFirstPixelHue %= 180;

  // Actualizar LEDs en cada ciclo
  led.setData(ledBuffer);


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
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 255, 0); // negro (apagado)
  }
  led.setData(ledBuffer);
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

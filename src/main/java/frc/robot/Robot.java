//Folder de Proyecto
package frc.robot;

//Framework de Robot FRC
import edu.wpi.first.wpilibj.TimedRobot;

//Utilidades generales
import edu.wpi.first.wpilibj.Timer;
import java.util.List;

//SmartDashboard 
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Notificaciones Elastic
import frc.robot.util.Elastic;

//Energia y PDP
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

//Motores y controladores
import edu.wpi.first.wpilibj.Servo;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

//Utilidades de matematicas
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.filter.LinearFilter;

//Joystick PS4
import edu.wpi.first.wpilibj.PS4Controller;

//Drive Mecanum y FOD
import edu.wpi.first.wpilibj.drive.MecanumDrive;

//Alertas Dashboards
import edu.wpi.first.wpilibj.Alert;

//DriverStation
import edu.wpi.first.wpilibj.DriverStation;

//Sensores Digitales y analogicos
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

//Sensores I2C y Color
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

//Sensores CAN CTRE
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

//Vision WebCam
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

//Leds Direccionables
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

//Tejuino Board
import frc.robot.TejuinoBoard;

//Clase principal de Robot heredando framework TimedRobot

public class Robot extends TimedRobot {

  // Creacion de objeto de sensor CANrange
  private final CANBus kCANBus = new CANBus("rio");
  private final CANrange canRange = new CANrange(10, kCANBus);

  // Creacion de objeto de entrada digigtal como Counter
  private final Counter counter = new Counter(Counter.Mode.kTwoPulse);

  // Creacion de objeto de sensor de Color Rev
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  // Creacion de objeto Leds Direccionables
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private int rainbowFirstPixelHue = 0;

  //Creacion de objeto Tejuino Board
  private final TejuinoBoard tejuino_board = new TejuinoBoard();

  // Creacion de objeto Menu selector de Autonomo
  private static final String kDefaultAuto = "Defaul Auto";
  private static final String kCustomAuto = "Line Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  Timer cronos = new Timer(); // Nuevo timer para cronometrar tiempo de autonomo.;

  // Creacion de objeto Motores y Drive Mecanum
  private final SparkMax frontLeftMotor = new SparkMax(5, MotorType.kBrushed);
  private final SparkMax rearLeftMotor = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax frontRightMotor = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax rearRightMotor = new SparkMax(2, MotorType.kBrushed);

  private final SparkMaxConfig frontLeftMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig rearLeftMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig frontRightMotorConfig = new SparkMaxConfig();
  private final SparkMaxConfig rearRightMotorConfig = new SparkMaxConfig();

  private final MecanumDrive mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);

  // Creacion de objeto Controlador PS4
  private final PS4Controller driverController = new PS4Controller(0);

  // Creacion de objeto Navx
  private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

  // Creacion de objeto PDP
  private final PowerDistribution PowerDistribution = new PowerDistribution(1, ModuleType.kCTRE);

  // Creacion de objeto Field 2D
  private final Field2d m_field = new Field2d();

  // Creacion de objeto Alertas
  Alert alert = new Alert("Modo FOD ACTIVADO", Alert.AlertType.kInfo);
  Alert alert2 = new Alert("PARO DE EMERGENCIA ACTIVADO", Alert.AlertType.kError);

  // Creacion de objeto de Servo
  private Servo intakeServo = new Servo(0); // Servo para el mecanismo de intake.

  // Creacion de objeto Sensores Digitales
  DigitalInput magneticSensor = new DigitalInput(4); // Sensor magnético en el eje X del robot.
  DigitalInput limitSwitch = new DigitalInput(6); // Sensor de limite
  DigitalInput InductiveSensor = new DigitalInput(7); // Sensor de limite

  // Creacion de objeto de sensores analogicos
  AnalogPotentiometer Ultrasonic = new AnalogPotentiometer(0, 5000, 300);

  // Creacion de objeto Acelerometro
  BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  // Creacion de objeto Notificaciones Elastic
  Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.INFO, "Teleoperado iniciado",
      "El modo teleoperado inicio correctamente");

  // Creacion de objeto Encoder
  Encoder encoder4x = new Encoder(0, 1, true, Encoder.EncodingType.k4X);

  // Creacion de objeto Encoder Absoluto
  DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(2, 360, 0);

  // Variables globales
  private boolean fod; // Habilitar o deshabilitar el control Field Oriented Drive.

  // Metodo de inicializacion del robot.
  public void robotInit() {

    // Configuracion de sensor CanRange
    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000; // If CANrange has a signal strength of at least 2000 its valid.
    config.ProximityParams.ProximityThreshold = 0.2; // If CANrange detects an object within 0.2 meters, it will trigger
    config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz; // Make the CANrange update as fast as possible at
    canRange.getConfigurator().apply(config);

    // Configuracion de Counter
    counter.reset();
    counter.setUpSource(9);
    counter.clearDownSource();
    counter.setUpSourceEdge(true, false);

    // Configuracion de Leds Direccionables
    led = new AddressableLED(6);
    ledBuffer = new AddressableLEDBuffer(5);// Crea buffer de 5 LEDs
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);// Asigna buffer al objeto LED
    led.start();// Activa la señal

    //Configuracion Tejuino Board
    tejuino_board.init(0);

    // Apagar todos al inicio
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 0); // R, G, B
    }
    led.setData(ledBuffer);

    // Configuracion de Menu selector de Autonomo
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Line 3S Auto", kCustomAuto);
    SmartDashboard.putData("Auto choserr", m_chooser);

    // Configuracion de Motores y Drive Mecanum
    frontLeftMotorConfig.inverted(true).idleMode(IdleMode.kBrake);
    rearLeftMotorConfig.inverted(true).idleMode(IdleMode.kBrake);
    frontRightMotorConfig.inverted(false).idleMode(IdleMode.kBrake);
    rearRightMotorConfig.inverted(false).idleMode(IdleMode.kBrake);
    frontLeftMotor.configure(frontLeftMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    rearLeftMotor.configure(rearLeftMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    frontRightMotor.configure(frontRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    rearRightMotor.configure(rearRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    mecanumDrive.setDeadband(0.02);
    mecanumDrive.setMaxOutput(1.0);

    // Configuracion de encoders
    encoder4x.setSamplesToAverage(5);
    encoder4x.setDistancePerPulse(1.0 / 360 * Math.PI * 6);
    encoder4x.setMinRate(1);
    encoder4x.reset();

    // Configuracion de Trayectorias
    Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    SmartDashboard.putData(m_field);

    m_field.getObject("traj").setTrajectory(m_trajectory);

    // Configuracion de Posicion inicial servo
    intakeServo.setAngle(90);

    // Inicia la captura automática de la primera cámara USB encontrada
    UsbCamera camera = CameraServer.startAutomaticCapture();
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

    // Lectura de sensor de color
    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    // Lectura del sensor de Proximidad
    int proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putString("Color Sensor", m_colorSensor.getColor().toHexString());

    
  }

  /** This function is called once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    // Selecciona el modo de autonomo
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    // Reset de cronometro y Navx
    cronos.start();
    cronos.reset();
    navx.reset();
    counter.reset();

    tejuino_board.all_leds_blue(0);

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
    counter.reset();
    fod = true;
    SmartDashboard.putBoolean("FOD", fod);
    SmartDashboard.putNumber("Servo Angle", 90);
    SmartDashboard.putData("Chasis", mecanumDrive);
    SmartDashboard.putData("PDP", PowerDistribution);

    Elastic.sendNotification(notification);

    //Control Leds Tejuino
    tejuino_board.all_leds_green(0);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    /* VARIABLES LOCALES TELEOP */

    // Lectura de tiempo de partido
    double matchTime = DriverStation.getMatchTime();

    // Lectura de angulo del servo desde SmartDashboard
    double servoIncrement = SmartDashboard.getNumber("Servo Angle", 90);
    intakeServo.setAngle(servoIncrement);

    // Actualizacion de alertas DASHBOARD
    alert.setText("Modo Fiel Oriented Drive ACTIVADO");
    alert.set(fod);
    alert2.set(DriverStation.isEStopped());

    // Efecto arcoiris en los LEDs
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;

    led.setData(ledBuffer);

    

    // Filtro para suavizar lectura del acelerometro
    LinearFilter xAccFilter = LinearFilter.movingAverage(10);

    // Escritura de datos en SmartDashboard
    SmartDashboard.putNumber("Accelerometro", xAccFilter.calculate(navx.getWorldLinearAccelX()));
    SmartDashboard.putNumber("Counter", counter.get());
    SmartDashboard.putData("Controller", driverController);
    SmartDashboard.putData("Navx Angle", navx);
    SmartDashboard.putNumber("Navx Yaw", navx.getYaw());
    SmartDashboard.putNumber("Match Time", matchTime);
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putBoolean("FOD", fod);
    SmartDashboard.putBoolean("Magnetic Sensor", magneticSensor.get());
    SmartDashboard.putNumber("Ultrasonico", Ultrasonic.get());
    SmartDashboard.putData("Rio Acelerometro", accelerometer);
    SmartDashboard.putNumber("Encoder en Distancia", Math.round(encoder4x.getDistance() * 100) / 100d);
    SmartDashboard.putData("Encoder Relativo", encoder4x);
    SmartDashboard.putData("Encoder Absoluto", absoluteEncoder);
    SmartDashboard.putBoolean("Limit switch", limitSwitch.get());
    SmartDashboard.putBoolean("Sensor Inductivo", InductiveSensor.get());
    SmartDashboard.putNumber("Temperatura PDP", PowerDistribution.getTemperature());
    SmartDashboard.putData("CANrange", canRange);

    /* CONTROL DE CHASIS MECANUM DRIVE CON FOD */

    // Lectura y escritura de FOD desde SmartDashboard
    fod = SmartDashboard.getBoolean("FOD", fod);
    // Lectura de ejes del Joystick PS4
    double ySpeed = -driverController.getLeftY(); // Remember, this is reversed!
    double xSpeed = driverController.getLeftX(); // Counteract imperfect strafing
    double zRotation = driverController.getRightX();

    if (fod) {
      Rotation2d navXAngle = Rotation2d.fromDegrees(navx.getAngle());
      // POV
      mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation, navXAngle);
      // navx FOD
    } else {
      mecanumDrive.driveCartesian(ySpeed, xSpeed, zRotation);
      // Sin navx
    }

    if (driverController.getOptionsButton() == true) {
      navx.reset();
    }    

    // Reset del counter con el limit switch
    if (limitSwitch.get() == false) {
      counter.reset();
    }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

    // Poner Leds en rojo al deshabilitar el robot
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 255, 0, 0); // Rojo en disabled
    }
    led.setData(ledBuffer);

    tejuino_board.rainbow_effect(0);

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

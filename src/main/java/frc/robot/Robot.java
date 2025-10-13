//Folder principal del Proyecto del robot
package frc.robot;

//Datalog Manager para registro de eventos en roborio
import edu.wpi.first.wpilibj.DataLogManager;

//Framework de Robot Timed FRC
import edu.wpi.first.wpilibj.TimedRobot;

//Utilidades generales de wpilib
import edu.wpi.first.wpilibj.Timer;
import java.util.List;

//Clases de utilidad SmartDashboard 
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//Librerias externas para telemetria y alertas y control de leds.
import frc.robot.util.Elastic;
import frc.robot.util.TejuinoBoard;

//Monitoreo de Energia y PDP
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

//Motor Drivers y controladores PID SparkMax
import edu.wpi.first.wpilibj.Servo; //Smart Robot Servo Rev Robotics
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;


//Utilidades de matematicas y trayectorias
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.filter.LinearFilter;

//Joystick PS4
import edu.wpi.first.wpilibj.PS4Controller;

//Drive Mecanum y FOD
import edu.wpi.first.wpilibj.drive.MecanumDrive;

//Alertas para envio a Dashboards
import edu.wpi.first.wpilibj.Alert;

//Obtencion de variables de DriverStation
import edu.wpi.first.wpilibj.DriverStation;

//Sensores Digitales y analogicos
import com.studica.frc.AHRS; //Sensor Navx Attitude and Heading Reference System (AHRS)
import edu.wpi.first.wpilibj.AnalogPotentiometer; //Sensor Ultrasonico Maxbotics
import edu.wpi.first.wpilibj.AnalogTrigger; //Analog Trigger Ultrasonico Maxbotics
import edu.wpi.first.wpilibj.BuiltInAccelerometer;// Acelerometro interno del RoboRIO
import edu.wpi.first.wpilibj.Counter; //Contador de game pieces sensor Photoelectrico
import edu.wpi.first.wpilibj.DigitalInput;//Limits Switches y Sensores Digitales
import edu.wpi.first.wpilibj.DutyCycleEncoder;//Sensor Encoder Absoluto Rev Through Bore Encoder
import edu.wpi.first.wpilibj.Encoder; //Sensor E4T OEM Miniature Optical Encoder

//Sensores I2C y Color
import edu.wpi.first.wpilibj.I2C; //Rev Color Sensor V3
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

//Sensores CAN CTRE
import com.ctre.phoenix6.CANBus; //Sensor de rango y proximidad CANrange
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

//Vision WebCam Microsoft HD 3000
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

//Leds Direccionables 5v WS2812, WS2812B, and WS2815
import edu.wpi.first.wpilibj.AddressableLED; 
import edu.wpi.first.wpilibj.AddressableLEDBuffer;


//Clase principal de Robot heredando framework TimedRobot
public class Robot extends TimedRobot {

  // Creacion de objeto de sensor de distancia y deteccion de objetos CANrange
  private final CANBus kCANBus = new CANBus("rio");
  private final CANrange canRange = new CANrange(10, kCANBus);

  // Creacion de objeto de entrada digital como Contador de piezas.
  private final Counter counter = new Counter(Counter.Mode.kTwoPulse);

  // Creacion de objeto de sensor de Color Rev
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  // Creacion de objeto Leds Direccionables
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private int rainbowFirstPixelHue = 0;

  //Creacion de objeto Leds Driver Tejuino Board
  private final TejuinoBoard tejuino_board = new TejuinoBoard();

  // Creacion de objeto Menu selector de Autonomo
  private static final String kDefaultAuto = "Default Auto";
  private static final String kCustomAuto = "Line Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Creacion de objeto Timer para retardo de autonomo
  Timer cronos = new Timer(); 

  //Creacion de objeto de Shooter
  private final SparkMax shooterMotor = new SparkMax(11, MotorType.kBrushless);
  private final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();
  private final SparkClosedLoopController shooterPid = shooterMotor.getClosedLoopController();

  //Creacion de objeto de Climber
  private final SparkMax climberMotor = new SparkMax(12, MotorType.kBrushless);
  private final SparkMaxConfig climberMotorConfig = new SparkMaxConfig();
  private final SoftLimitConfig climberSoftLimitsConfig = new SoftLimitConfig();
  private final SparkClosedLoopController climberPid = climberMotor.getClosedLoopController();

  // Creacion de objetos Motores de Drive Mecanum
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

  // Creacion de objeto de giroscopio y AHRS Navx
  private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

  // Creacion de objeto PDP
  private final PowerDistribution PowerDistribution = new PowerDistribution(1, ModuleType.kCTRE);

  // Creacion de objeto Field 2D
  private final Field2d m_field = new Field2d();

  // Creacion de objeto Alertas Dashboard
  Alert alert = new Alert("Modo FOD ACTIVADO", Alert.AlertType.kInfo);
  Alert alert2 = new Alert("PARO DE EMERGENCIA ACTIVADO", Alert.AlertType.kError);
  Alert noAutoSelected = new Alert("No se selecciono modo autonomo", Alert.AlertType.kWarning);

  // Creacion de objeto de Servomotor REV
  private Servo intakeServo = new Servo(0); // Servo para el mecanismo de intake.

  // Creacion de objeto Sensores Digitales
  DigitalInput magneticSensor = new DigitalInput(4); // Sensor magnético Rev Magnetic Limit Switch
  DigitalInput limitSwitch = new DigitalInput(6); // Sensor Micro Limit Switch 
  DigitalInput InductiveSensor = new DigitalInput(7); // Sensor Inductivo Npn Lj12a3- 4-z/bx

  // Creacion de objeto de sensores analogicos ultrasonicos
  AnalogPotentiometer Ultrasonic = new AnalogPotentiometer(0, 5000, 300);

  // Creacion de objeto Analog Trigger para usar señar boleana de deteccion del sensor ultrasonico 
  AnalogTrigger ultrasonicTrigger = new AnalogTrigger(1);

  // Creacion de objeto Acelerometro interno del RoboRIO
  BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

  // Creacion de objeto Notificaciones internas de dashboard Elastic
  Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.INFO, "Teleoperado iniciado",
      "El modo teleoperado inicio correctamente");
  
  Elastic.Notification autoSelectedNotification = new Elastic.Notification(Elastic.NotificationLevel.INFO, "Autonomo seleccionado",
      "El modo autonomo seleccionado es: " + m_autoSelected);

  // Creacion de objeto Encoder Relativo incremental
  Encoder encoder4x = new Encoder(0, 1, true, Encoder.EncodingType.k4X);

  // Creacion de objeto Encoder Absoluto Rev
  DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(2, 360, 0);

  // Variables globales para todo el robot
  private boolean fod; // Habilitar o deshabilitar el control Field Oriented Drive.
  private double climberSetPoint; // Variable para almacenar el setpoint del climber.
  boolean ClimberEnablePID = false; // Variable para habilitar o deshabilitar el control PID del climber
  private double shooterSetPoint = 0;//Variable para almacenar el setpoint del shooter

  

  //Creacion de objeto de Sendable personalizado  del Shooter PID Sparkmax para envio a elastic.
  Sendable pidShooterSendable = new Sendable() {
    @Override
    public void initSendable(SendableBuilder shooterBuilder) {
      shooterBuilder.setSmartDashboardType("Shooter PIDController");

      shooterBuilder.addDoubleProperty("P", () -> shooterMotor.configAccessor.closedLoop.getP(), 
      x -> {shooterMotorConfig.closedLoop.p(x);
            shooterMotor.configure(shooterMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      shooterBuilder.addDoubleProperty("I", () -> shooterMotor.configAccessor.closedLoop.getI(),
      x -> {shooterMotorConfig.closedLoop.i(x);
            shooterMotor.configure(shooterMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      shooterBuilder.addDoubleProperty("D", () -> shooterMotor.configAccessor.closedLoop.getD(),
      x -> {shooterMotorConfig.closedLoop.d(x);
            shooterMotor.configure(shooterMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      shooterBuilder.addDoubleProperty("FF", () -> shooterMotor.configAccessor.closedLoop.getFF(),
      x -> {shooterMotorConfig.closedLoop.velocityFF(x);
            shooterMotor.configure(shooterMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });
    }
  };
  // Creacion de objeto de Sendable personalizado del Climber PID Sparkmax para envio a elastic.
  Sendable pidClimberSendable = new Sendable() {
    @Override
    public void initSendable(SendableBuilder climberBuilder) {
      climberBuilder.setSmartDashboardType("Climber PIDController");

      climberBuilder.addDoubleProperty("P", () -> climberMotor.configAccessor.closedLoop.getP(),
      x -> {climberMotorConfig.closedLoop.p(x);
            climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      climberBuilder.addDoubleProperty("I", () -> climberMotor.configAccessor.closedLoop.getI(),
      x -> {climberMotorConfig.closedLoop.i(x);
            climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      climberBuilder.addDoubleProperty("D", () -> climberMotor.configAccessor.closedLoop.getD(),
      x -> {climberMotorConfig.closedLoop.d(x);
            climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });

      climberBuilder.addDoubleProperty("FF", () -> climberMotor.configAccessor.closedLoop.getFF(),
      x -> {climberMotorConfig.closedLoop.velocityFF(x);
            climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
      });
    }
  };
  

  // Metodo de inicializacion del robot.
  public void robotInit() {

    // Inicia el DataLogManager en el roborio
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    //Configuracion de analog trigger del sensor ultrasonico
    double voltagePercent = 5.0 / 5000.0; // 5V corresponde a 5000mm
    ultrasonicTrigger.setLimitsVoltage(500*voltagePercent,800*voltagePercent);// Setea los limites de trigger entre 500mm y 800mm
    ultrasonicTrigger.setFiltered(true);// Habilita el filtro para evitar ruido
        
    // Configuracion de sensor CanRange
    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000; // If CANrange has a signal strength of at least 2000 its valid.
    config.ProximityParams.ProximityThreshold = 0.2; // If CANrange detects an object within 0.2 meters, it will trigger
    config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz; // Make the CANrange update as fast as possible at
    canRange.getConfigurator().apply(config);// Apply the configuration to the CANrange

    // Configuracion de Counter
    counter.setUpSource(9);//
    counter.reset();//Reset del contador
    counter.clearDownSource();// No se usa fuente de conteo negativo
    counter.setUpSourceEdge(true, false);//Conteo valido solo para flanco de subida.

    // Configuracion de Leds Direccionables
    led = new AddressableLED(6);// Crea objeto LED en puerto PWM 6
    ledBuffer = new AddressableLEDBuffer(5);// Crea buffer de 5 LEDs
    led.setLength(ledBuffer.getLength());// Asigna largo del buffer al objeto LED
    led.setData(ledBuffer);// Asigna buffer al objeto LED
    led.start();// Activa la señal para los LEDs

    // Apagar todos los leds al inicio
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 0); // R, G, B
    }
    led.setData(ledBuffer);


    //Configuracion Tejuino Board
    tejuino_board.init(0);

    // Configuracion de Menu selector de Autonomo
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Line 3 Segundos Auto", kCustomAuto);
    SmartDashboard.putData("Auto choserr", m_chooser);

    // Configuracion de motor de Shooter
    //final double shooterVelocityFF = 1 / 5676; // El valor FF es el inverso de la maxima velocidad en RPM del motor

    shooterMotorConfig.idleMode(IdleMode.kCoast);
    shooterMotorConfig.inverted(true);
    shooterMotorConfig.smartCurrentLimit(40);
    shooterMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    shooterMotorConfig.closedLoop.pidf(0.000001,0,0,0.000179); //Valor FF encontrado con sintonizacion manual    
    shooterMotorConfig.closedLoop.outputRange(-1, 1);
    shooterSetPoint = 0;

    shooterMotor.configure(shooterMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    // Configuracion de motor de Climber
    climberMotorConfig.idleMode(IdleMode.kBrake);
    climberMotorConfig.inverted(true);
    climberMotorConfig.smartCurrentLimit(40);
    climberMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    climberMotorConfig.closedLoop.pid(0.1,0,0.001);
    climberMotorConfig.closedLoop.outputRange(-1, 1);
    climberSetPoint = 0;
    //Configuracion de Soft Limits del Climber
    climberSoftLimitsConfig.forwardSoftLimitEnabled(true);
    climberSoftLimitsConfig.forwardSoftLimit(50);
    climberSoftLimitsConfig.reverseSoftLimitEnabled(true);
    climberSoftLimitsConfig.reverseSoftLimit(0);

    climberMotorConfig.apply(climberSoftLimitsConfig);// Aplica configuracion de soft limits al motor
    climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);// Aplica configuracion al motor
    climberMotor.getEncoder().setPosition(0);// Reset de posicion del encoder al iniciar


    // Configuracion de Motores y Drive Mecanum
    frontLeftMotorConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    rearLeftMotorConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    frontRightMotorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    rearRightMotorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    frontLeftMotor.configure(frontLeftMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    rearLeftMotor.configure(rearLeftMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    frontRightMotor.configure(frontRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    rearRightMotor.configure(rearRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    mecanumDrive.setDeadband(0.02);// Zona muerta del joystick
    mecanumDrive.setMaxOutput(1.0);// Maximo output del drive
    mecanumDrive.setSafetyEnabled(true);// Habilita el safety del drive
    mecanumDrive.setExpiration(0.1);// Tiempo de expiracion del safety

    

    // Configuracion de encoders
    encoder4x.setSamplesToAverage(10);
    encoder4x.setDistancePerPulse(1.0 / 360 * (Math.PI * 6)); // 360 pulsos por vuelta por 4x, rueda de 6 pulgadas, reduccion 10:1 y paso a pulgadas
    encoder4x.setMinRate(10);
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

   
    if (m_chooser.getSelected() == kDefaultAuto) {
      noAutoSelected.set(true);
      } else {
      noAutoSelected.set(false);
    }

    // Lectura de sensor de color
    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();
    int proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putString("Color Sensor", m_colorSensor.getColor().toHexString());
    
  }
  /** This function is called once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {

    // Selecciona el modo de autonomo
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
    Elastic.sendNotification(autoSelectedNotification);

    

    // Reset de Timer y Navx
    cronos.start();
    cronos.reset();
    navx.reset();
    counter.reset();

    //Control de leds en canales 0 y 1 del tejuino board.
    tejuino_board.all_leds_blue(0);
    tejuino_board.all_leds_blue(1);

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    switch (m_autoSelected) {
      case kDefaultAuto:
        //No Hace nada el robot en autonomo
        break;
      case kCustomAuto:

        if (cronos.get() <= 3) {
          mecanumDrive.driveCartesian(0.5, 0, 0);
        } else {
          mecanumDrive.driveCartesian(0, 0, 0);
        }
        break;
      default:
    }
    // Put default auto code here }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    // Reset de Timer y Navx
    navx.reset();
    counter.reset();
    fod = true;

    // Escritura de datos iniciales en SmartDashboard
    SmartDashboard.putBoolean("FOD", fod);
    SmartDashboard.putNumber("Servo Angle", 90);
    SmartDashboard.putData("Chasis", mecanumDrive);
    SmartDashboard.putData("PDP", PowerDistribution);

    //Envio de Controles PID de Shooter via sendable SmartDashboard
    SmartDashboard.putData("PID Shooter", pidShooterSendable); 
    //Envio de Controles PID de Climber via sendable SmartDashboard
    SmartDashboard.putData("PID Climber", pidClimberSendable);
          
       
    // Notificacion de inicio de teleop
    Elastic.sendNotification(notification);

    //Control Leds Tejuino
    tejuino_board.all_leds_green(0);
    tejuino_board.all_leds_green(1);

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

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

    // Filtro para suavizar lectura del acelerometro del navx
    LinearFilter xAccFilter = LinearFilter.movingAverage(10);

    // Escritura de datos en SmartDashboard
    SmartDashboard.putNumber("Accelerometro X Navx", xAccFilter.calculate(navx.getWorldLinearAccelX()));
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
    SmartDashboard.putNumber("Encoder en Distancia", Math.round(encoder4x.getDistance() * 100) / 100d);// Distancia en pulgadas con 2 decimales
    SmartDashboard.putData("Encoder Relativo", encoder4x);
    SmartDashboard.putData("Encoder Absoluto", absoluteEncoder);
    SmartDashboard.putBoolean("Limit switch", limitSwitch.get());
    SmartDashboard.putBoolean("Sensor Inductivo", InductiveSensor.get());
    SmartDashboard.putNumber("Temperatura PDP", PowerDistribution.getTemperature());
    SmartDashboard.putData("CANrange", canRange);
    SmartDashboard.putNumber("Climber Position Encoder", climberMotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("ultrasonic trigger", ultrasonicTrigger.getTriggerState());

    //PID Climber Smartdashboard
    SmartDashboard.putNumber("Climber Set Point", climberSetPoint);
    SmartDashboard.putNumber("Climber Encoder", climberMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber Output", climberMotor.getAppliedOutput());

    //PID Shooter Smartdashboard
    SmartDashboard.putNumber("Shooter Set Point", shooterSetPoint);
    SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Output", shooterMotor.getAppliedOutput());
    

    /* CONTROL DE CHASIS MECANUM DRIVE CON FOD */

    // Lectura y escritura de FOD desde SmartDashboard
    fod = SmartDashboard.getBoolean("FOD", fod);
    // Lectura de ejes del Joystick PS4
    double xSpeed = -driverController.getLeftY(); // Remember, this is reversed!
    double ySpeed = driverController.getLeftX(); 
    double zRotation = driverController.getRightX();

    // Boton para activar o desactivar FOD
    if (fod) {
      Rotation2d navXAngle = Rotation2d.fromDegrees(navx.getAngle());
      // POV
      mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation, navXAngle);
      // navx FOD
    } else {
      mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation);
      // Sin navx
    }

    // Reset del Navx con el boton OPTIONS del PS4
    if (driverController.getOptionsButton() == true) {
      navx.reset();
    }    

    // Reset del counter con el limit switch
    if (limitSwitch.get() == false) {
      counter.reset();
    }

    /*CONTROL DEL SHOOTER*/

  // Control del shooter con botones PS4
  if (driverController.getR2Button()) {
    shooterSetPoint= 0; //Detiene el Shooter con PID

  } else if (driverController.getL2Button()) {
    shooterSetPoint = 3000; //Arranca el shooter con PID
  } 

  shooterPid.setReference(shooterSetPoint, ControlType.kVelocity); // Control PID

  //Control de climber con PS4 y PID
  if (driverController.getL1Button() || driverController.getR1Button()) {
    ClimberEnablePID = false;
  }

  if (driverController.getCrossButton() || driverController.getCircleButton() || driverController.getTriangleButton()) {
    ClimberEnablePID = true; // Habilita PID si se usan los botones de posicion
  }
  
  //Control de climber con botones PS4 L1 y R1
  if (!ClimberEnablePID) {
    if (driverController.getL1Button()) {
      climberMotor.set(0.5); // Sube el climber
    } else if (driverController.getR1Button()) {
      climberMotor.set(-0.5); // Baja el climber
    } else {
      climberMotor.set(0); // Detiene el climber
    }
  }

  // Control del climber con PID usando botones PS4
  if (ClimberEnablePID) {
    if (driverController.getCrossButton()) {
      climberSetPoint = 0; // Posición 0
    } else if (driverController.getCircleButton()) {
      climberSetPoint = 25; // Posición 25
    } else if (driverController.getTriangleButton()) {
      climberSetPoint = 50; // Posición 50
    }

    climberPid.setReference(climberSetPoint, ControlType.kPosition); // Control PID
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
    tejuino_board.rainbow_effect(1);

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

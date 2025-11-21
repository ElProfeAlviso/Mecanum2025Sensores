/**
 * Este es un codigo de ejemplo de como programar los distintos sensores y motores (NEOS y Brushed) de un robot FRC
 * utilizando la libreria RevSpark y CTRE CANrange junto con el framework WPILib FRC y visualizarlos en Elastic.
 * @author El Profe Alviso (Titanium Rams 5959)
 * @version 1.0.0
 * @date Noviembre 2025
 * @since Java 17
 */




//Folder principal del Proyecto del robot
package frc.robot;

//Datalog Manager para registro de eventos en roborio directamente
import edu.wpi.first.wpilibj.DataLogManager;

//Clase para uso de Framework de Robot Timed FRC
import edu.wpi.first.wpilibj.TimedRobot;

//Utilidades generales de wpilib timer para cronometros y list para listas de datos.
import edu.wpi.first.wpilibj.Timer;
import java.util.List;

//Clases de utilidad SmartDashboard 
import edu.wpi.first.wpilibj.smartdashboard.Field2d; //Campo 2D para visualizacion de trayectorias
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;// Selector de opciones Auto en dashboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;//Utilidad para envio de datos a dashboard (Elastic

//Librerias externas para telemetria y alertas y control de leds.
import frc.robot.util.Elastic; //Utilidades para envio de alertas a dashboard Elastic
import frc.robot.util.TejuinoBoard;//Utilidades para control de Tejuino Board y Leds

// Importación de clases para monitoreo de energía y Power Distribution Panel (PDP)
import edu.wpi.first.wpilibj.PowerDistribution; // Clase para interactuar con el PDP
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType; // Enumeración para especificar el tipo de módulo PDP

// Importación de librerías para control de motores SparkMax y configuración de PID
import edu.wpi.first.wpilibj.Servo; // Librería para el control de servomotores Rev Robotics
import com.revrobotics.spark.SparkBase; // Clase base para controladores Spark
import com.revrobotics.spark.SparkMax; // Clase para controladores SparkMax
import com.revrobotics.spark.SparkBase.ControlType; // Tipos de control para Spark (ej. posición, velocidad)
import com.revrobotics.spark.SparkLowLevel.MotorType; // Tipos de motores compatibles con SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode; // Modos de inactividad (freno o libre)
import com.revrobotics.spark.config.SoftLimitConfig; // Configuración de límites suaves por encoder para motores
import com.revrobotics.spark.config.SparkMaxConfig; // Configuración general para SparkMax
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor; // Sensores de retroalimentación para control en lazo cerrado
import com.revrobotics.spark.SparkClosedLoopController; // Controlador PID para SparkMax

// Importación de utilidades matemáticas y trayectorias
import edu.wpi.first.math.geometry.Pose2d; // Representación de posición en 2D (x, y, rotación)
import edu.wpi.first.math.geometry.Rotation2d; // Representación de rotación en 2D
import edu.wpi.first.math.geometry.Translation2d; // Representación de traslación en 2D
import edu.wpi.first.math.trajectory.Trajectory; // Clase para trayectorias de movimiento
import edu.wpi.first.math.trajectory.TrajectoryConfig; // Configuración de trayectorias (velocidad, aceleración)
import edu.wpi.first.math.trajectory.TrajectoryGenerator; // Generador de trayectorias
import edu.wpi.first.math.util.Units; // Utilidades para conversión de unidades (ej. metros a pies)
import edu.wpi.first.util.sendable.Sendable; // Interfaz para enviar datos a SmartDashboard
import edu.wpi.first.util.sendable.SendableBuilder; // Constructor para objetos Sendable
import edu.wpi.first.math.filter.LinearFilter; // Filtro lineal para suavizar datos

// Importación de clase para el control del Joystick PS4
import edu.wpi.first.wpilibj.PS4Controller;

// Importación de clase para el control de un chasis Mecanum y Field Oriented Drive (FOD)
import edu.wpi.first.wpilibj.drive.MecanumDrive;

// Importación de clase para generar alertas y enviarlas a los Dashboards
import edu.wpi.first.wpilibj.Alert;

// Importación de clase para obtener variables y datos del DriverStation
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
import edu.wpi.first.wpilibj.I2C; // Librería para comunicación I2C, utilizada por el sensor de color Rev Color Sensor V3
import edu.wpi.first.wpilibj.util.Color; // Clase para representar colores detectados por el sensor
import com.revrobotics.ColorSensorV3; // Librería para el sensor de color Rev Color Sensor V3

//Sensores CAN CTRE
import com.ctre.phoenix6.CANBus; // Librería para comunicación CAN, utilizada por el sensor de rango y proximidad CANrange
import com.ctre.phoenix6.configs.CANrangeConfiguration; // Configuración para el sensor CANrange
import com.ctre.phoenix6.hardware.CANrange; // Clase para el sensor de rango y proximidad CANrange
import com.ctre.phoenix6.signals.UpdateModeValue; // Enumeración para los modos de actualización del sensor CANrange

//Visión WebCam Microsoft HD 3000
import edu.wpi.first.cameraserver.CameraServer; // Librería para iniciar la captura de video desde cámaras USB
import edu.wpi.first.cscore.UsbCamera; // Clase para manejar cámaras USB

//Leds Direccionables 5v WS2812, WS2812B, and WS2815
import edu.wpi.first.wpilibj.AddressableLED; // Librería para controlar LEDs direccionables
import edu.wpi.first.wpilibj.AddressableLEDBuffer; // Buffer para almacenar los datos de los LEDs direccionables


//Clase principal de Robot heredando framework TimedRobot
public class Robot extends TimedRobot {

  // Creacion de objeto de sensor de distancia y deteccion de objetos CANrange
  private final CANBus kCANBus = new CANBus("rio"); // Bus CAN para comunicación con dispositivos
  private final CANrange canRange = new CANrange(10, kCANBus); // Sensor de rango CANrange en ID 10

  // Creacion de objeto de entrada digital como Contador de piezas.
  private final Counter counter = new Counter(Counter.Mode.kTwoPulse); // Contador para detección de piezas

  // Creacion de objeto de sensor de Color Rev
  private final I2C.Port i2cPort = I2C.Port.kOnboard; // Puerto I2C para el sensor de color
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort); // Sensor de color Rev

  // Creacion de objeto Leds Direccionables
  private AddressableLED led; // Objeto para controlar LEDs direccionables
  private AddressableLEDBuffer ledBuffer; // Buffer para almacenar datos de LEDs
  private int rainbowFirstPixelHue = 0; // Variable para efecto arcoiris en LEDs

  // Creacion de objeto Leds Driver Tejuino Board
  private final TejuinoBoard tejuino_board = new TejuinoBoard(); // Controlador de LEDs Tejuino Board

  // Creacion de objeto Menu selector de Autonomo
  private static final String kDefaultAuto = "Default Auto"; // Opción por defecto para autónomo
  private static final String kCustomAuto = "Line Auto"; // Opción personalizada para autónomo
  private String m_autoSelected; // Variable para almacenar la opción seleccionada
  private final SendableChooser<String> m_chooser = new SendableChooser<>(); // Menú selector de autónomo

  // Creacion de objeto Timer para retardo de autonomo
  Timer cronos = new Timer(); // Temporizador para tareas de autónomo

  // Creacion de objeto de Shooter
  private final SparkMax shooterMotor = new SparkMax(11, MotorType.kBrushless); // Motor del shooter
  private final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig(); // Configuración del motor del shooter
  private final SparkClosedLoopController shooterPid = shooterMotor.getClosedLoopController(); // Controlador PID del shooter

  // Creacion de objeto de Climber
  private final SparkMax climberMotor = new SparkMax(12, MotorType.kBrushless); // Motor del climber
  private final SparkMaxConfig climberMotorConfig = new SparkMaxConfig(); // Configuración del motor del climber
  private final SoftLimitConfig climberSoftLimitsConfig = new SoftLimitConfig(); // Configuración de límites suaves del climber
  private final SparkClosedLoopController climberPid = climberMotor.getClosedLoopController(); // Controlador PID del climber

  // Creacion de objetos Motores de Drive Mecanum
  private final SparkMax frontLeftMotor = new SparkMax(5, MotorType.kBrushed); // Motor delantero izquierdo
  private final SparkMax rearLeftMotor = new SparkMax(3, MotorType.kBrushed); // Motor trasero izquierdo
  private final SparkMax frontRightMotor = new SparkMax(4, MotorType.kBrushed); // Motor delantero derecho
  private final SparkMax rearRightMotor = new SparkMax(2, MotorType.kBrushed); // Motor trasero derecho

  private final SparkMaxConfig frontLeftMotorConfig = new SparkMaxConfig(); // Configuración del motor delantero izquierdo
  private final SparkMaxConfig rearLeftMotorConfig = new SparkMaxConfig(); // Configuración del motor trasero izquierdo
  private final SparkMaxConfig frontRightMotorConfig = new SparkMaxConfig(); // Configuración del motor delantero derecho
  private final SparkMaxConfig rearRightMotorConfig = new SparkMaxConfig(); // Configuración del motor trasero derecho

  private final MecanumDrive mecanumDrive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor); // Objeto para control de chasis mecanum

  // Creacion de objeto Controlador PS4
  private final PS4Controller driverController = new PS4Controller(0); // Controlador PS4 en puerto 0

  // Creacion de objeto de giroscopio y AHRS Navx
  private final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI); // Giroscopio Navx conectado por SPI

  // Creacion de objeto PDP
  private final PowerDistribution PowerDistribution = new PowerDistribution(1, ModuleType.kCTRE); // Panel de distribución de energía

  // Creacion de objeto Field 2D
  private final Field2d m_field = new Field2d(); // Objeto para visualización del campo en 2D

  // Creacion de objeto Alertas Dashboard
  Alert alert = new Alert("Modo FOD ACTIVADO", Alert.AlertType.kInfo); // Alerta informativa para FOD
  Alert alert2 = new Alert("PARO DE EMERGENCIA ACTIVADO", Alert.AlertType.kError); // Alerta de error para paro de emergencia
  Alert noAutoSelected = new Alert("No se selecciono modo autonomo", Alert.AlertType.kWarning); // Alerta de advertencia para autónomo no seleccionado

  // Creacion de objeto de Servomotor REV
  private Servo intakeServo = new Servo(0); // Servo para el mecanismo de intake en puerto PWM 0

  // Creacion de objeto Sensores Digitales
  DigitalInput magneticSensor = new DigitalInput(4); // Sensor magnético en puerto digital 4
  DigitalInput limitSwitch = new DigitalInput(6); // Sensor de límite en puerto digital 6
  DigitalInput InductiveSensor = new DigitalInput(7); // Sensor inductivo en puerto digital 7

  // Creacion de objeto de sensores analogicos ultrasonicos
  AnalogPotentiometer Ultrasonic = new AnalogPotentiometer(0, 5000, 300); // Sensor ultrasónico en puerto analógico 0

  // Creacion de objeto Analog Trigger para usar señal boleana de deteccion del sensor ultrasonico 
  AnalogTrigger ultrasonicTrigger = new AnalogTrigger(1); // Trigger analógico en puerto analógico 1

  // Creacion de objeto Acelerometro interno del RoboRIO
  BuiltInAccelerometer accelerometer = new BuiltInAccelerometer(); // Acelerómetro interno del RoboRIO

  // Creacion de objeto Notificaciones internas de dashboard Elastic
  Elastic.Notification notification = new Elastic.Notification(Elastic.NotificationLevel.INFO, "Teleoperado iniciado",
      "El modo teleoperado inicio correctamente"); // Notificación para inicio de teleoperado
  
  Elastic.Notification autoSelectedNotification = new Elastic.Notification(Elastic.NotificationLevel.INFO, "Autonomo seleccionado",
      "El modo autonomo seleccionado es: " + m_autoSelected); // Notificación para autónomo seleccionado

  // Creacion de objeto Encoder Relativo incremental
  Encoder encoder4x = new Encoder(0, 1, true, Encoder.EncodingType.k4X); // Encoder incremental en puertos digitales 0 y 1

  // Creacion de objeto Encoder Absoluto Rev
  DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(2, 360, 0); // Encoder absoluto en puerto digital 2

  // Variables globales para todo el robot
  private boolean fod; // Habilitar o deshabilitar el control Field Oriented Drive.
  private double climberSetPoint; // Variable para almacenar el setpoint del climber.
  boolean ClimberEnablePID = false; // Variable para habilitar o deshabilitar el control PID del climber
  private double shooterSetPoint = 0;//Variable para almacenar el setpoint del shooter
  private int dashboardCounter = 0; // Contador para controlar la frecuencia de actualizacion del dashboard

  

  //Creacion de objeto de Sendable personalizado  del Shooter PID Sparkmax para envio a elastic.
  //Esto crea un objeto en el dashboard que permite modificar los valores del PID en tiempo real.
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
  // Esto crea un objeto en el dashboard que permite modificar los valores del PID en tiempo real.
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
    CANrangeConfiguration config = new CANrangeConfiguration(); // Crea objeto de configuracion del CANrange
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
    // Apagar todos los leds al inicio para evitar que queden encendidos con valores previos
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 0); // Establece el color de los LEDs en negro (apagado)
    }
    led.setData(ledBuffer); // Actualiza los LEDs con los valores del buffer

    // Configuración inicial de la Tejuino Board
    tejuino_board.init(0); // Inicializa la Tejuino Board en el canal 0

    // Configuración del menú selector de modo autónomo
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto); // Establece la opción por defecto como "Default Auto"
    m_chooser.addOption("Line 3 Segundos Auto", kCustomAuto); // Agrega una opción personalizada "Line 3 Segundos Auto"
    SmartDashboard.putData("Auto choserr", m_chooser); // Publica el menú selector en el SmartDashboard

    // Configuracion de motor de Shooter
    // Configura el modo de inactividad, inversión, límite de corriente y sensor de retroalimentación
    shooterMotorConfig.idleMode(IdleMode.kCoast); //Configura el modo Libre sin freno
    shooterMotorConfig.inverted(true);//Invierte el giro del motor
    shooterMotorConfig.smartCurrentLimit(40);//Establece el límite de corriente
    shooterMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);// Usa el encoder interno como sensor de retroalimentación
    shooterMotorConfig.closedLoop.pidf(0.000001, 0, 0, 0.000179); // Valores PID y FF ajustados manualmente
    shooterMotorConfig.closedLoop.outputRange(-1, 1); // Rango de salida del controlador PID
    shooterSetPoint = 0; // Setpoint inicial del shooter

    // Aplica la configuración al motor del shooter
    shooterMotor.configure(shooterMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    // Configuracion de motor de Climber
    climberMotorConfig.idleMode(IdleMode.kBrake); // Configura el modo de inactividad en freno
    climberMotorConfig.inverted(true); // Invierte el giro del motor
    climberMotorConfig.smartCurrentLimit(40); // Establece el límite de corriente
    climberMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder); // Usa el encoder interno como sensor de retroalimentación
    climberMotorConfig.closedLoop.pid(0.1, 0, 0.001); // Valores PID ajustados manualmente (Usando Rev Hardware Client
    climberMotorConfig.closedLoop.outputRange(-1, 1); // Rango de salida del controlador PID
    climberSetPoint = 0; // Setpoint inicial del climber

    // Configuración de límites suaves (soft limits) del Climber
    climberSoftLimitsConfig.forwardSoftLimitEnabled(true); // Habilita límite suave hacia adelante
    climberSoftLimitsConfig.forwardSoftLimit(50); // Posición máxima hacia adelante
    climberSoftLimitsConfig.reverseSoftLimitEnabled(true); // Habilita límite suave hacia atrás
    climberSoftLimitsConfig.reverseSoftLimit(0); // Posición mínima hacia atrás

    // Aplica la configuración de límites suaves y del motor
    climberMotorConfig.apply(climberSoftLimitsConfig);
    climberMotor.configure(climberMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    climberMotor.getEncoder().setPosition(0); // Resetea la posición del encoder al iniciar

    // Configuracion de Motores y Drive Mecanum
    // Configura cada motor del chasis mecanum
    //inversion de giro, modo de inactividad y limite de corriente
    frontLeftMotorConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    rearLeftMotorConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    frontRightMotorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);
    rearRightMotorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

    // Aplica la configuración a cada motor
    frontLeftMotor.configure(frontLeftMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    rearLeftMotor.configure(rearLeftMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    frontRightMotor.configure(frontRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    rearRightMotor.configure(rearRightMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

    // Configura el chasis mecanum
    mecanumDrive.setDeadband(0.02); // Zona muerta del joystick
    mecanumDrive.setMaxOutput(1.0); // Salida máxima del drive
    mecanumDrive.setSafetyEnabled(true); // Habilita el sistema de seguridad
    mecanumDrive.setExpiration(0.1); // Tiempo de expiración del sistema de seguridad

    // Configuracion de encoders
    encoder4x.setSamplesToAverage(10); // Promedia 10 muestras para suavizar la lectura
    encoder4x.setDistancePerPulse(1.0 / 360 * (Math.PI * 6)); // Configura la distancia por pulso en pulgadas
    encoder4x.setMinRate(10); // Configura la tasa mínima de pulsos
    encoder4x.reset(); // Resetea el encoder

    // Configuracion de Trayectorias
    // Genera una trayectoria con puntos intermedios y velocidades máximas
    Trajectory m_trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
      new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    // Publica la trayectoria en el SmartDashboard
    SmartDashboard.putData(m_field);
    m_field.getObject("traj").setTrajectory(m_trajectory);

    // Configuracion de Posicion inicial servo
    intakeServo.setAngle(90); // Establece el ángulo inicial del servo

    // Inicia la captura automática de la primera cámara USB encontrada
    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(320, 240); // Configura la resolución de la cámara
    camera.setFPS(15); // Configura los cuadros por segundo
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
    // Verifica si el modo autónomo seleccionado es el predeterminado
    if (m_chooser.getSelected() == kDefaultAuto) {
      // Activa la alerta si no se seleccionó un modo autónomo diferente
      noAutoSelected.set(true);
    } else {
      // Desactiva la alerta si se seleccionó un modo autónomo válido
      noAutoSelected.set(false);
    }
  }
  /** This function is called once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {

    // Selecciona el modo de autónomo desde el menú desplegable en el SmartDashboard
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);

    // Envía una notificación a Elastic con el modo autónomo seleccionado
    Elastic.sendNotification(autoSelectedNotification);

    // Reinicia y comienza el temporizador para medir el tiempo en el modo autónomo
    cronos.start();
    cronos.reset();

    // Reinicia el giroscopio Navx para establecer el ángulo inicial en 0
    navx.reset();

    // Reinicia el contador de piezas detectadas
    counter.reset();

    // Cambia el color de los LEDs en los canales 0 y 1 de la Tejuino Board a azul
    tejuino_board.all_leds_blue(0);
    tejuino_board.all_leds_blue(1);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    // Verifica el modo autónomo seleccionado
    switch (m_autoSelected) {
      case kDefaultAuto:
        // En el modo predeterminado, el robot no realiza ninguna acción en autónomo
        break;
      case kCustomAuto:
        // En el modo personalizado, el robot avanza en línea recta durante 3 segundos
        if (cronos.get() <= 3) {
          mecanumDrive.driveCartesian(0.5, 0, 0); // Avanza hacia adelante con velocidad 0.5
        } else {
          mecanumDrive.driveCartesian(0, 0, 0); // Detiene el robot después de 3 segundos
        }
        break;
      default:
        // Código para otros modos autónomos (si se agregan en el futuro)
    }
    // Fin del código autónomo predeterminado
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    // Resetea el giroscopio Navx y el contador de piezas detectadas
    navx.reset();
    counter.reset();

    // Habilita el modo Field Oriented Drive (FOD) al inicio del teleoperado
    fod = true;

    // Inicializa valores en el SmartDashboard
    SmartDashboard.putBoolean("FOD", fod); // Publica el estado inicial de FOD
    SmartDashboard.putNumber("Servo Angle", 90); // Publica el ángulo inicial del servo
    SmartDashboard.putData("Chasis", mecanumDrive); // Publica el objeto del chasis mecanum
    SmartDashboard.putData("PDP", PowerDistribution); // Publica el objeto del panel de distribución de energía

    // Envía los controles PID del Shooter al SmartDashboard para ajustes en tiempo real
    SmartDashboard.putData("PID Shooter", pidShooterSendable); 
    // Envía los controles PID del Climber al SmartDashboard para ajustes en tiempo real
    SmartDashboard.putData("PID Climber", pidClimberSendable);
          
    // Envía una notificación a Elastic indicando que el modo teleoperado ha iniciado
    Elastic.sendNotification(notification);

    // Cambia los LEDs de la Tejuino Board a verde para indicar que el robot está en teleoperado
    tejuino_board.all_leds_green(0);
    tejuino_board.all_leds_green(1);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    // Lectura de sensor de color
    Color detectedColor = m_colorSensor.getColor(); // Obtiene el color detectado por el sensor
    double IR = m_colorSensor.getIR(); // Obtiene el valor de infrarrojo detectado
    int proximity = m_colorSensor.getProximity(); // Obtiene la proximidad detectada por el sensor

    // Lectura de tiempo de partido
    double matchTime = DriverStation.getMatchTime(); // Obtiene el tiempo restante del partido

    // Lectura de ángulo del servo desde SmartDashboard
    double servoIncrement = SmartDashboard.getNumber("Servo Angle", 90); // Obtiene el ángulo deseado desde el dashboard
    intakeServo.setAngle(servoIncrement); // Ajusta el ángulo del servo al valor obtenido

    // Actualización de alertas en el Dashboard
    alert.setText("Modo Field Oriented Drive ACTIVADO"); // Actualiza el texto de la alerta
    alert.set(fod); // Activa o desactiva la alerta según el estado de FOD
    alert2.set(DriverStation.isEStopped()); // Activa la alerta si el robot está en paro de emergencia

    // Efecto arcoiris en los LEDs
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      final int hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180; // Calcula el tono para cada LED
      ledBuffer.setHSV(i, hue, 255, 128); // Establece el color del LED en el buffer
    }
    rainbowFirstPixelHue += 3; // Incrementa el tono inicial para el efecto arcoiris
    rainbowFirstPixelHue %= 180; // Asegura que el tono esté dentro del rango válido

    led.setData(ledBuffer); // Actualiza los LEDs con los valores del buffer

    // Filtro para suavizar lectura del acelerómetro del Navx
    LinearFilter xAccFilter = LinearFilter.movingAverage(10); // Crea un filtro de promedio móvil con 10 muestras

    // Actualización del dashboard cada 100ms aproximadamente
    dashboardCounter++; // Incrementa el contador de ciclos

    if (dashboardCounter % 10 == 0) { // Actualiza el dashboard cada 10 ciclos (~200ms)

    // Escritura y envio de datos de visualizacion en SmartDashboard Elastic
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

    // Color Sensor SmartDashboard
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    SmartDashboard.putNumber("Proximity", proximity);
    SmartDashboard.putString("Color Sensor", m_colorSensor.getColor().toHexString());


    //PID Climber Smartdashboard
    SmartDashboard.putNumber("Climber Set Point", climberSetPoint);
    SmartDashboard.putNumber("Climber Encoder", climberMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Climber Output", climberMotor.getAppliedOutput());

    //PID Shooter Smartdashboard
    SmartDashboard.putNumber("Shooter Set Point", shooterSetPoint);
    SmartDashboard.putNumber("Shooter Velocity", shooterMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Shooter Output", shooterMotor.getAppliedOutput());
  }

    /* CONTROL DE CHASIS MECANUM DRIVE CON FOD */

    // Lectura y escritura de FOD desde SmartDashboard
    fod = SmartDashboard.getBoolean("FOD", fod); // Obtiene el estado de FOD desde el SmartDashboard

    // Lectura de ejes del Joystick PS4
    double xSpeed = -driverController.getLeftY(); // Eje Y del joystick izquierdo (invertido)
    double ySpeed = driverController.getLeftX(); // Eje X del joystick izquierdo
    double zRotation = driverController.getRightX(); // Eje X del joystick derecho para rotación

    // Botón para activar o desactivar FOD (Field Oriented Drive)
    if (fod) {
      Rotation2d navXAngle = Rotation2d.fromDegrees(navx.getAngle()); // Obtiene el ángulo del Navx
      mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation, navXAngle); // Control con FOD activado
    } else {
      mecanumDrive.driveCartesian(xSpeed, ySpeed, zRotation); // Control sin FOD
    }

    // Reset del Navx con el botón OPTIONS del PS4
    if (driverController.getOptionsButton() == true) {
      navx.reset(); // Resetea el ángulo del Navx
    }    

    // Reset del contador con el limit switch
    if (limitSwitch.get() == false) {
      counter.reset(); // Resetea el contador si el limit switch está presionado
    }

    /* CONTROL DEL SHOOTER */

    // Control del shooter con botones PS4
    if (driverController.getR2Button()) {
      shooterSetPoint = 0; // Detiene el shooter con PID
    } else if (driverController.getL2Button()) {
      shooterSetPoint = 3000; // Arranca el shooter con PID a 3000 RPM
    }

    shooterPid.setReference(shooterSetPoint, ControlType.kVelocity); // Control PID para el shooter

    /* CONTROL DEL CLIMBER */

    // Control del climber con botones PS4 L1 y R1 sin PID
    if (driverController.getL1Button() || driverController.getR1Button()) {
      ClimberEnablePID = false; // Desactiva el control PID si se usan L1 o R1
    }

    // Habilita el control PID del climber si se usan los botones de posición
    if (driverController.getCrossButton() || driverController.getCircleButton() || driverController.getTriangleButton()) {
      ClimberEnablePID = true; // Activa el control PID
    }

    // Control del climber sin PID usando botones PS4 L1 y R1
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

      climberPid.setReference(climberSetPoint, ControlType.kPosition); // Control PID para el climber
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {

    // Cambia el color de los LEDs direccionables a rojo al deshabilitar el robot
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 255, 0, 0); // Establece el color rojo para cada LED
    }
    led.setData(ledBuffer); // Actualiza los LEDs con los valores del buffer

    // Activa el efecto arcoiris en los LEDs de la Tejuino Board en los canales 0 y 1
    tejuino_board.rainbow_effect(0); // Efecto arcoiris en el canal 0
    tejuino_board.rainbow_effect(1); // Efecto arcoiris en el canal 1

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

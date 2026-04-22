package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Swerve.DriveTrain;

public class ShooterTestSubsystem extends SubsystemBase {

    // -------------------------------------------------------------------------
    // Donanım
    // -------------------------------------------------------------------------
    private final AnalogInput hoodPotInput = new AnalogInput(ShooterConstants.hoodPotPort);  // Analog port 2
    private final VictorSP    hoodMotor    = new VictorSP(ShooterConstants.hoodMotorPWM);     // PWM port 1


    private final SparkMax masterNeo = new SparkMax(ShooterConstants.masterNeoCanID, MotorType.kBrushless);
    private final SparkMax follower1 = new SparkMax(ShooterConstants.follower1NeoCanID, MotorType.kBrushless);
    private final SparkMax follower2 = new SparkMax(ShooterConstants.follower2NeoCanID, MotorType.kBrushless);
    private final SparkClosedLoopController shooterPID;
    private final InterpolatingDoubleTreeMap velocityTable = new InterpolatingDoubleTreeMap();
    private double targetRPM = 0;

    private boolean isShooterRunning = false; 


    // -------------------------------------------------------------------------
    // Swerve
    // -------------------------------------------------------------------------
    private final DriveTrain m_driveTrain;

    // -------------------------------------------------------------------------
    // Interpolasyon Tablosu — Mesafe(m) → Hood Açısı(°)
    // -------------------------------------------------------------------------
    private final InterpolatingDoubleTreeMap hoodMap     = new InterpolatingDoubleTreeMap();
    private final Translation2d             kHubLocation = new Translation2d(4.76 - 1.24, 4.11);

    // -------------------------------------------------------------------------
    // Potansiyometre Sabitleri
    // -------------------------------------------------------------------------
    private static final double kMinPotValue     = 4.0; 
    private static final double kMaxPotValue     = 4007.0;
    private static final double kPotTotalDegrees = 270.0;

    // -------------------------------------------------------------------------
    // Hood Açısı Offset — İlerleyen testlerde buradan ayarla
    // -------------------------------------------------------------------------
    private static final double kHoodAngleOffset = 30.0; // derece

    // -------------------------------------------------------------------------
    // PID
    //   kP = 0.05  → Titreşirse düşür (0.02 dene)
    //   kI = 0.0   → Şimdilik sıfır bırak
    //   kD = 0.001 → Overshoot varsa artır
    // -------------------------------------------------------------------------
    private final PIDController hoodPID = new PIDController(0.1, 0.0, 0.001);

    private static final double kAngleTolerance = 1.5; // ± derece tolerans

    // -------------------------------------------------------------------------
    // Durum
    // -------------------------------------------------------------------------
    private boolean pidEnabled = false;

    // =========================================================================
    // Constructor
    // =========================================================================
    public ShooterTestSubsystem(DriveTrain driveTrain) {
        this.m_driveTrain = driveTrain;
        this.shooterPID = masterNeo.getClosedLoopController();

        hoodMotor.setInverted(true);

        hoodPID.setTolerance(kAngleTolerance);

        

        setupMap();
        configureShooterMotors();
    }

    private void configureShooterMotors() {
        SparkMaxConfig config = new SparkMaxConfig();
        
        config.closedLoop
            .p(0.00042)
            .i(0)
            .d(0.00004)
            .outputRange(-1, 1);

        config.inverted(true);
            
        
        config.closedLoop.feedForward.kV(0.000203);

        config.closedLoopRampRate(0.1);
        
        masterNeo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(11); // Master ID'yi takip et
        follower1.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower2.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // =========================================================================
    // Interpolasyon Tablosu
    // =========================================================================
    private void setupMap() {
        // put(Mesafe_Metre, Gereken_Hood_Acisi_Derece)
        hoodMap.put(1.0,  3.0);
        hoodMap.put(2.0,  6.0);
        hoodMap.put(3.0, 15.70);
        hoodMap.put(4.0, 18.5);
        hoodMap.put(5.0, 30.0);
        hoodMap.put(6.0, 30.0);
        hoodMap.put(7.0, 30.0);

        // Atış hızı tablosu (mesafe m → RPM)
        velocityTable.put(1.0, 2700.0);
        velocityTable.put(2.0, 2700.0);
        velocityTable.put(3.0, 3000.0);
        velocityTable.put(4.0, 3200.0);
        velocityTable.put(5.0, 3500.0);
        velocityTable.put(6.0, 3700.0);
        velocityTable.put(7.0, 4500.0);
    }

    // =========================================================================
    // Sensör Okuma
    // =========================================================================

    /**
     * Potansiyometrenin ham değerini derece cinsine çevirir.
     * kHoodAngleOffset uygulanır. Çıkış 0.0 – 270.0 arasında sınırlandırılmıştır.
     */
    public double getHoodCurrentAngle() {
        double raw   = (double) hoodPotInput.getValue();
        double angle = (raw - kMinPotValue) * kPotTotalDegrees / (kMaxPotValue - kMinPotValue);
        angle -= kHoodAngleOffset; // Offset uygula
        return angle;
    }

    /**
     * Odometri konumuna göre hub'a olan mesafeyi metre cinsinden döndürür.
     */
    public double getDistanceToHub() {
        Pose2d currentPose = m_driveTrain.getPose();
        return currentPose.getTranslation().getDistance(kHubLocation);
    }

    /**
     * Mesafeye göre interpolasyon tablosundan hedef hood açısını döndürür.
     */
    public double getTargetAngle() {
        return hoodMap.get(getDistanceToHub());
    }

    // =========================================================================
    // PID Kontrol
    // =========================================================================

    /**
     * PID'i etkinleştirir — hood otomatik olarak hedef açıya gider.
     * Shooter komutu başlarken çağır.
     */
    public void enablePID() {
        hoodPID.setSetpoint(getTargetAngle());
        pidEnabled = true;
    }

    /**
     * PID'i devre dışı bırakır ve motoru durdurur.
     * Komut bitişinde çağır.
     */
    public void disablePID() {
        pidEnabled = false;
        hoodMotor.set(0.0);
        hoodPID.reset();
    }

    /**
     * Hood'un hedef açıya ulaşıp ulaşmadığını döndürür (± kAngleTolerance).
     * Ateşlemeden önce bu değeri kontrol et.
     */
    public boolean isAtTarget() {
        return hoodPID.atSetpoint();
    }

    /**
     * Manuel motor kontrolü — test ve debug için.
     * Çağrıldığında PID'i otomatik olarak devre dışı bırakır.
     *
     * @param speed -1.0 ile 1.0 arasında motor hızı
     */
    public void setHoodSpeed() {
        disablePID();
        hoodMotor.set(0.3);
    }

    // =========================================================================
    // Periodic
    // =========================================================================
    @Override
    public void periodic() {
        double currentAngle = getHoodCurrentAngle();
        double targetAngle  = getTargetAngle();
        double distance     = getDistanceToHub();
        double error        = targetAngle - currentAngle;

        // PID aktifse motoru sürdür
        if (pidEnabled) {
            // Robot hareket ettikçe mesafe değişebilir, setpoint'i güncelle
            hoodPID.setSetpoint(targetAngle);

            double output = hoodPID.calculate(currentAngle);

            // Çıkışı güvenli aralıkta tut
            output = Math.max(-0.5, Math.min(0.5, output));

            hoodMotor.set(output);
        }

        targetRPM = velocityTable.get(distance);
        double currentRPM = masterNeo.getEncoder().getVelocity();
        boolean isShooterReady = Math.abs(currentRPM - targetRPM) < 50.0;
        boolean isHoodReady = Math.abs(getHoodCurrentAngle() - targetAngle) < 1.5;
        
        // Hem hız hem açı istenen aralıktaysa atışa hazırız
        boolean isShootReady = isShooterReady && isHoodReady;


        // SmartDashboard
        SmartDashboard.putNumber("Hood/Mevcut Aci",       currentAngle);
        SmartDashboard.putNumber("Hood/Hedef Aci",        targetAngle);
        SmartDashboard.putNumber("Hood/Hub Mesafesi (m)", distance);
        SmartDashboard.putNumber("Hood/Hata (°)",         error);
        SmartDashboard.putNumber("Hood/Motor Cikisi",     hoodMotor.get());
        SmartDashboard.putNumber("Hood/Offset (°)",       kHoodAngleOffset);
        SmartDashboard.putBoolean("Hood/Hedefe Ulasti",   isAtTarget());
        SmartDashboard.putBoolean("Hood/PID Aktif",       pidEnabled);
        SmartDashboard.putBoolean("Shooter/isShootReady", isShootReady);
        SmartDashboard.putBoolean("Shooter/Hiz Tamam", isShooterReady);
        SmartDashboard.putNumber("Shooter/Hiz", currentRPM);
        SmartDashboard.putNumber("Shooter/Hedef Hiz", targetRPM);
    }

    public void shoot() {
        shooterPID.setSetpoint(targetRPM, SparkMax.ControlType.kVelocity);
    }

    public void stopShooter() {
        isShooterRunning = false; 
        //shooterPID.setSetpoint(0, SparkMax.ControlType.kVelocity);
        masterNeo.set(0);
    }

    public boolean isReady() {
        double currentRPM = masterNeo.getEncoder().getVelocity();
        double currentPotValue = getHoodCurrentAngle();
        
        boolean isShooterReady = Math.abs(currentRPM - targetRPM) < 50.0;
        boolean isHoodReady = Math.abs(currentPotValue - getTargetAngle()) < 1.5;
        
        return isShooterReady && isHoodReady;
    }

    public void toggleShooter() {
        isShooterRunning = !isShooterRunning; // Durumu tersine çevir

        if (isShooterRunning) {
            // Hedef RPM'e sür
            shooterPID.setSetpoint(targetRPM, SparkMax.ControlType.kVelocity);
        } else {
            // Motorları durdur
            shooterPID.setSetpoint(0, SparkMax.ControlType.kVelocity);
            // Alternatif olarak masterNeo.stopMotor(); da diyebilirsin
        }
    }
    public void setShooterSpesific() {
        hoodPID.setSetpoint(15.70);
        shooterPID.setSetpoint(2500, SparkMax.ControlType.kVelocity);
    }
}
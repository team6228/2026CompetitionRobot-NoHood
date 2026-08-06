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
    // Mod Enum
    // -------------------------------------------------------------------------
    public enum ShootMode {
        INTERPOLATION,  // Hood 0°'de, mesafeye göre interpolasyon
        PRESET_A,       // Sabit: 15.70° / 2500 RPM
        PRESET_B        // Sabit: 45° / 3000 RPM  ← buraya kendi değerini yaz
    }

    private ShootMode currentMode = ShootMode.INTERPOLATION;

    // -------------------------------------------------------------------------
    // Preset Sabitleri  ← Bu değerleri istediğin gibi değiştir
    // -------------------------------------------------------------------------
    // private static final double kPresetA_Angle = 15.70;
    // private static final double kPresetA_RPM   = 2500.0;

    // private static final double kPresetB_Angle = 45.0;
    // private static final double kPresetB_RPM   = 4500.0;

    // -------------------------------------------------------------------------
    // Donanım
    // -------------------------------------------------------------------------
    private final AnalogInput hoodPotInput = new AnalogInput(ShooterConstants.hoodPotPort);
    private final VictorSP    hoodMotor    = new VictorSP(ShooterConstants.hoodMotorPWM);

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
    // private final InterpolatingDoubleTreeMap hoodMap      = new InterpolatingDoubleTreeMap();
    private final Translation2d             kHubLocation  = new Translation2d(5.76 - 1.24, 4.11);

    // -------------------------------------------------------------------------
    // Potansiyometre Sabitleri
    // -------------------------------------------------------------------------
    private static final double kMinPotValue     = 4.0;
    private static final double kMaxPotValue     = 4007.0;
    private static final double kPotTotalDegrees = 270.0;

    // -------------------------------------------------------------------------
    // Hood Açısı Offset
    // -------------------------------------------------------------------------
    private static final double kHoodAngleOffset = 30.0;

    // -------------------------------------------------------------------------
    // PID
    // -------------------------------------------------------------------------
    private final PIDController hoodPID = new PIDController(0.1, 0.0, 0.001);

    private static final double kAngleTolerance = 1.5;

    // -------------------------------------------------------------------------
    // Durum
    // -------------------------------------------------------------------------
    private boolean pidEnabled = false;

    // =========================================================================
    // Constructor
    // =========================================================================
    public ShooterTestSubsystem(DriveTrain driveTrain) {
        this.m_driveTrain = driveTrain;
        this.shooterPID   = masterNeo.getClosedLoopController();

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
        followConfig.follow(11);
        follower1.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower2.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // =========================================================================
    // Interpolasyon Tablosu
    // =========================================================================
    private void setupMap() {
        velocityTable.put(1.0, 2700.0);
        velocityTable.put(2.0, 2700.0);
        velocityTable.put(3.0, 3000.0);
        velocityTable.put(4.0, 3200.0);
        velocityTable.put(5.0, 3500.0);
        velocityTable.put(6.0, 3700.0);
        velocityTable.put(7.0, 4500.0);

        // hoodMap.put(1.0,  3.0);
        // hoodMap.put(2.0,  6.0);
        // hoodMap.put(3.0, 15.70);
        // hoodMap.put(4.0, 18.5);
        // hoodMap.put(5.0, 30.0);
        // hoodMap.put(6.0, 30.0);
        // hoodMap.put(7.0, 30.0);

    }

    // =========================================================================
    // Sensör Okuma
    // =========================================================================
    public double getHoodCurrentAngle() {
        double raw   = (double) hoodPotInput.getValue();
        double angle = (raw - kMinPotValue) * kPotTotalDegrees / (kMaxPotValue - kMinPotValue);
        angle -= kHoodAngleOffset;
        return angle;
    }

    public double getDistanceToHub() {
        Pose2d currentPose = m_driveTrain.getPose();
        return currentPose.getTranslation().getDistance(kHubLocation);
    }

    public double getTargetAngle() {
        // return hoodMap.get(getDistanceToHub());
        return 55.0;
    }

    // =========================================================================
    // Mod Yönetimi
    // =========================================================================

    /**
     * A tuşu toggle:
     *   - PRESET_A değilse → PRESET_A'ya gir, hood 15.70°'ye git
     *   - PRESET_A ise     → INTERPOLATION'a dön, hood 0°'ye git
     */
    // public void togglePresetA() {
    //     if (currentMode == ShootMode.PRESET_A) {
    //         currentMode = ShootMode.INTERPOLATION;
    //         hoodPID.setSetpoint(0.0);
    //     } else {
    //         currentMode = ShootMode.PRESET_A;
    //         hoodPID.setSetpoint(kPresetA_Angle);
    //     }
    //     pidEnabled = true;
    // }

    // /**
    //  * B tuşu toggle:
    //  *   - PRESET_B değilse → PRESET_B'ye gir, hood 45°'ye git
    //  *   - PRESET_B ise     → INTERPOLATION'a dön, hood 0°'ye git
    //  */
    // public void togglePresetB() {
    //     if (currentMode == ShootMode.PRESET_B) {
    //         currentMode = ShootMode.INTERPOLATION;
    //         hoodPID.setSetpoint(0.0);
    //     } else {
    //         currentMode = ShootMode.PRESET_B;
    //         hoodPID.setSetpoint(kPresetB_Angle);
    //     }
    //     pidEnabled = true;
    // }

    // public ShootMode getCurrentMode() {
    //     return currentMode;
    // }

    // =========================================================================
    // PID Kontrol
    // =========================================================================

    /**
     * Mevcut moda göre doğru açıyı setpoint olarak ayarlar ve PID'i başlatır.
     */
    public void enablePID() {
        // switch (currentMode) {
            // case PRESET_A:
            //     hoodPID.setSetpoint(kPresetA_Angle);
            //     break;
            // case PRESET_B:
            //     hoodPID.setSetpoint(kPresetB_Angle);
            //     break;
            // case INTERPOLATION:
            // default:
        // hoodPID.setSetpoint(getTargetAngle());
        //         break;
        // }
        pidEnabled = true;
    }

    /**
     * PID'i devre dışı bırakır — motor periodic()'te idle konumuna (0°) döner.
     */
    public void disablePID() {
        pidEnabled = false;
        hoodPID.reset();
        // Modu INTERPOLATION'a sıfırla ki bir sonraki tetik interpolasyonla çalışsın
        // (sadece tetik bırakıldığında modu korumak istiyorsan bu satırı sil)
        // currentMode = ShootMode.INTERPOLATION;
    }

    public boolean isAtTarget() {
        return hoodPID.atSetpoint();
    }

    // public void setHoodSpeed() {
    //     disablePID();
    //     hoodMotor.set(0.3);
    // }

    // =========================================================================
    // Shooter Kontrol
    // =========================================================================

    /**
     * Mevcut moda göre doğru RPM'i hedefler ve shooterPID'i çalıştırır.
     */
    public void shoot() {
        targetRPM = velocityTable.get(getDistanceToHub());
        shooterPID.setSetpoint(targetRPM, SparkMax.ControlType.kVelocity);
        hoodPID.setSetpoint(55.0);
        pidEnabled=true;
    }

    public void stopShooter() {
        isShooterRunning = false;
        masterNeo.set(0);
    }

    /**
     * DPad Up için shooter toggle — mevcut moda göre RPM seçer.
     */
    public void toggleShooter() {
        isShooterRunning = !isShooterRunning;

        if (isShooterRunning) {
            targetRPM = velocityTable.get(getDistanceToHub());
            shooterPID.setSetpoint(targetRPM, SparkMax.ControlType.kVelocity);
        } else {
            shooterPID.setSetpoint(0, SparkMax.ControlType.kVelocity);
        }
    }

    /**
     * Hood PID aktifken hem hız hem açı toleransta mı?
     */
    public boolean isReady() {
        double currentRPM = masterNeo.getEncoder().getVelocity();
        
        boolean isShooterReady = Math.abs(currentRPM - targetRPM) < 50.0;

        return isShooterReady;
    }

    public void setShooterSpesific() {
        hoodPID.setSetpoint(15.70);
        shooterPID.setSetpoint(2500, SparkMax.ControlType.kVelocity);
    }

    // =========================================================================
    // Periodic
    // =========================================================================
    @Override
    public void periodic() {
        double currentAngle = getHoodCurrentAngle();
        double distance     = getDistanceToHub();

        if (pidEnabled) {
            hoodPID.setSetpoint(getTargetAngle());
            double output = hoodPID.calculate(currentAngle);
            output = Math.max(-0.5, Math.min(0.5, output));
            hoodMotor.set(output);

        } else {
            // Boşta: hood 0°'ye dön (yavaş)
            hoodPID.setSetpoint(0.0);
            double idleOutput = hoodPID.calculate(currentAngle);
            idleOutput = Math.max(-0.3, Math.min(0.3, idleOutput));
            hoodMotor.set(idleOutput);
        }

        // Shooter hazır mı?
        // targetRPM = switch (currentMode) {
        //     case PRESET_A       -> kPresetA_RPM;
        //     case PRESET_B       -> kPresetB_RPM;
        //     case INTERPOLATION  -> velocityTable.get(distance);
        // };
        targetRPM = velocityTable.get(distance);

        double currentRPM      = masterNeo.getEncoder().getVelocity();
        boolean isShooterReady = Math.abs(currentRPM - targetRPM) < 50.0;
        boolean isHoodReady    = isAtTarget();
        boolean isShootReady   = isShooterReady && isHoodReady;

        double targetAngle = hoodPID.getSetpoint();
        double error       = targetAngle - currentAngle;

        // SmartDashboard
        SmartDashboard.putNumber ("Hood/Mevcut Aci",        currentAngle);
        SmartDashboard.putNumber ("Hood/Hedef Aci",         targetAngle);
        SmartDashboard.putNumber ("Hood/Hub Mesafesi (m)",  distance);
        SmartDashboard.putNumber ("Hood/Hata (°)",          error);
        SmartDashboard.putNumber ("Hood/Motor Cikisi",      hoodMotor.get());
        SmartDashboard.putNumber ("Hood/Offset (°)",        kHoodAngleOffset);
        SmartDashboard.putBoolean("Hood/Hedefe Ulasti",     isAtTarget());
        SmartDashboard.putBoolean("Hood/PID Aktif",         pidEnabled);
        SmartDashboard.putBoolean("Shooter/isShootReady",   isShootReady);
        SmartDashboard.putBoolean("Shooter/Hiz Tamam",      isShooterReady);
        SmartDashboard.putNumber ("Shooter/Hiz",            currentRPM);
        SmartDashboard.putNumber ("Shooter/Hedef Hiz",      targetRPM);
        SmartDashboard.putString ("Shooter/Mod",            currentMode.toString());
    }
}
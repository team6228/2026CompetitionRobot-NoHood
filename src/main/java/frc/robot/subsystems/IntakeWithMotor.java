package frc.robot.subsystems;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeWithMotor extends SubsystemBase {
    
    private static final double GEAR_RATIO = 42.61;
    
    private static final double UP_POSITION_DEG   = -1.0;   
    private static final double DOWN_POSITION_DEG =  -142.50;

    private boolean isUp = true; // Başlangıçta aşağıda

    private static double degreesToRotations(double degrees) {
        return (degrees / 360.0) * GEAR_RATIO;
    }

    private static double rotationsToDegrees(double rotations) {
        return (rotations / GEAR_RATIO) * 360.0;
    }

    private final SparkMax intakeMotor = new SparkMax(13, MotorType.kBrushless);
    private final PWMSparkMax intakeTakeMotor = new PWMSparkMax(IntakeConstants.intakeMotorPWM);
    private final SparkClosedLoopController intakePID;
    private final SparkMaxConfig intakeConfig;

    public IntakeWithMotor() {
        this.intakeConfig = new SparkMaxConfig();
        this.intakePID    = intakeMotor.getClosedLoopController();

        intakeConfig.closedLoop
            .p(0.055)
            .i(0)
            .d(0.0001)
            .outputRange(-1, 1);

        // intakeConfig.softLimit
        //     .forwardSoftLimit((float) degreesToRotations(-95.0))
        //     .reverseSoftLimit((float) degreesToRotations(1.0))
        //     .forwardSoftLimitEnabled(true)
        //     .reverseSoftLimitEnabled(true);

        intakeConfig.inverted(true);
        intakeConfig.closedLoopRampRate(0.3);
        intakeConfig.idleMode(IdleMode.kCoast);

        intakeMotor.configure(
            intakeConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
        intakeMotor.getEncoder().setPosition(0);

        intakeTakeMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        double currentRotations = intakeMotor.getEncoder().getPosition();
        double currentDegrees   = rotationsToDegrees(currentRotations);

        SmartDashboard.putNumber("Intake Açısı (derece)", currentDegrees);
        SmartDashboard.putNumber("Intake Rotasyon",       currentRotations);
        // SmartDashboard.putBoolean("Forward Limit Hit", intakeMotor.getFaults().softLimitFwd);
        // SmartDashboard.putBoolean("Reverse Limit Hit", intakeMotor.getFaults().softLimitRev);
        SmartDashboard.putString("Faults", intakeMotor.getFaults().toString());
        SmartDashboard.putBoolean("isup", isUp);
        
    }

    public void upIntake() {
        isUp = true;
        intakePID.setSetpoint(
            degreesToRotations(UP_POSITION_DEG),
            ControlType.kPosition
        );
    }

    public void downIntake() {
        isUp = false;
        intakePID.setSetpoint(
            degreesToRotations(DOWN_POSITION_DEG),
            ControlType.kPosition
        );
    }

    public void toggleIntake() {
        if (isUp) {
            downIntake();
        } 
        else if (!isUp) {
            upIntake();
        }
        else {
            upIntake();
        }
    }

    public void setIntakeAngle(double intakeAngle) {
        intakePID.setSetpoint(
            degreesToRotations(intakeAngle),
            ControlType.kPosition
        );
    }

    public boolean isUp() {
        return isUp;
    }
    public void runIntake(){
       intakeTakeMotor.set(1);
     }

     public void stopIntake(){
        intakeTakeMotor.set(0);
     }
}
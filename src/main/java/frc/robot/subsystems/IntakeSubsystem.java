package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

    private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM, 
        IntakeConstants.intakeForwardChannel, 
        IntakeConstants.intakeReverseChannel
    );

    private final PWMSparkMax intakeMotor = new PWMSparkMax(IntakeConstants.intakeMotorPWM);

    public IntakeSubsystem(){

    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Intake Ready", intakeSolenoid.get() == DoubleSolenoid.Value.kForward);
    }

    public void openIntake(){
        intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void closeIntake(){
        intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void intakeToggle(){
        if (intakeSolenoid.get() == DoubleSolenoid.Value.kForward) {
            intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
        } else {
            intakeSolenoid.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void runIntake(){
        intakeMotor.set(0.8);
    }

    public void stopIntake(){
        intakeMotor.set(0);
    }
}
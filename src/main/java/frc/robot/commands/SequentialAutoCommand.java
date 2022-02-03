package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.classes.TargetPosition2D;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.AutonomousCommandConstants.*;
import frc.robot.Constants.AutonomousCommandConstants.StartPositions;





public class SequentialAutoCommand extends SequentialCommandGroup {

    public enum AutoStartPositions
    {
        LEFT,
        MIDDLE,
        RIGHT
    }
    private AutoStartPositions m_startPosition;
    private Drivetrain m_drivetrain;
    private Kinematics m_kinematics;
    
    
    public SequentialAutoCommand(Drivetrain drivetrain, Kinematics kinematics, AutoStartPositions startPosition) {
        
        m_drivetrain = drivetrain;
        m_kinematics = kinematics;
        m_startPosition = startPosition;
        SmartDashboard.putBoolean("AutoDone", false);
        
        switch (m_startPosition) {
            case LEFT:
                drivetrain.initAutonomous(new Position2D(0, 0, Math.toRadians(0)));    
                addCommands(  
                    new DriveTo(new TargetPosition2D(1, 0, Math.toRadians(0), 1.0d), m_kinematics, m_drivetrain),
                    new Wait(5),
                    new DriveTo(new TargetPosition2D(2, 1, Math.toRadians(0), 1.0d), m_kinematics, m_drivetrain)
                );
                break;
            case MIDDLE:

                break;
            case RIGHT:

                break;
            default:
                break;
        }
        addCommands(
            //Add commands to this list in order
        );
    }
}

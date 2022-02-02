package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.classes.Kinematics;
import frc.robot.classes.Position2D;
import frc.robot.classes.TargetPosition2D;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.AutonomousCommandConstants.*;

public class SequentialAutoCommand extends SequentialCommandGroup {
    public SequentialAutoCommand(Drivetrain drivetrain, Kinematics kinematics, startPositions startPosition) {
        SmartDashboard.putBoolean("AutoDone", false);
        
        switch (startPosition) {
            case LEFT:
                drivetrain.initAutonomous(new Position2D(0, 0, Math.toRadians(0)));    
                addCommands(  
                    new DriveTo(new TargetPosition2D(1, 0, Math.toRadians(0), 1.0d), kinematics, drivetrain),
                    new Wait(5),
                    new DriveTo(new TargetPosition2D(2, 1, Math.toRadians(0), 1.0d), kinematics, drivetrain)
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

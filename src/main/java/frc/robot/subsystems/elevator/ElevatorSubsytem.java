//package frc.robot.subsystems.elevator;
//
//import edu.wpi.first.math.filter.Debouncer;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//
//
//import java.util.function.DoubleSupplier;
//
//public class ElevatorSubsystem extends SubsystemBase {
//    public enum ElevatorGoal {
//        LEVELONE(()-> 0.1),
//        LEVELTWO(()-> 0.2),
//        LEVELTHREE(()-> 0.3),
//        IDLE(()-> 0.0);
//    }
//    private final DoubleSupplier heightSupplier;
//
//    public double getHeightSupplier() {
//        return heightSupplier.getAsDouble();
//    }
//}
//
//private ElevatorSubsystem.ElevatorGoal goal = ElevatorSubsystem.ElevatorGoal.IDLE;
//private Debouncer currentDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kFalling);
//}
//}

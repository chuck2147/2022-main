package frc.robot.util.interpolableMap;

public interface InverseInterpolable<T> {
    double inverseInterpolate(T upper, T query);
}

package frc.robot.util.interpolableMap;

public interface Interpolable<T> {
    T interpolate(T other, double t);
}

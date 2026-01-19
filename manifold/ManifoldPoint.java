package manifold;

public interface ManifoldPoint<T extends ManifoldPoint<T>> {
    T copy();
}

package manifold;

public interface TangentVector<P extends ManifoldPoint<P>, V extends TangentVector<P, V>> {
    P basePoint();
    V add(V other);
    V scale(double alpha);
    BilinearForm<V> tensorProduct(V other);

    default V subtract(V other) {
        return this.add(other.scale(-1));
    }
}
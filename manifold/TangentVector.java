package manifold;

public interface TangentVector<P extends ManifoldPoint<P>, V extends TangentVector<P, V>> {
    P basePoint();
    V add(V other);
    V scale(double alpha);
    Matrix<V> tensorProduct(V other);
}
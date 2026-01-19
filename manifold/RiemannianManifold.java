package manifold;

import java.util.function.BiFunction;
import java.util.function.Function;

public interface RiemannianManifold<P extends ManifoldPoint<P>, V extends TangentVector<P, V>> {
    BiFunction<V, V, Double> metric(P point);

    default double innerProduct(V vectorOne, V vectorTwo) {
        return metric(vectorOne.basePoint()).apply(vectorOne, vectorTwo);
    }

    default double norm(V vector) {
        return Math.sqrt(innerProduct(vector, vector));
    }

    default double dist(V vectorOne, V vectorTwo) {
        return norm(vectorOne.add(vectorTwo.scale(-1)));
    }

    P exp(P point, V vector);
    V log(P start, P end);
    V gradient(P point, Function<P, Double> function);
    V parallelTransport(P start, P end, V vector);
}

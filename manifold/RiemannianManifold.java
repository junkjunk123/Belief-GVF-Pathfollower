package manifold;

import manifold.endomorphism.Endomorphism;
import mathematics.DifferentiableScalarFunction;

import java.util.function.BiFunction;

public interface RiemannianManifold<P extends ManifoldPoint<P>, V extends TangentVector<P, V>, M extends Endomorphism<V>> {
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

    int dim();

    P exp(P point, V vector);
    V log(P start, P end);
    V gradient(P point, DifferentiableScalarFunction<P> function);
    V parallelTransport(P start, P end, V vector);

    default double quadraticForm(V vectorOne, M matrix, V vectorTwo) {
        V intermediate = matrix.multiply(vectorTwo);
        return innerProduct(vectorOne, intermediate);
    }

    default double quadraticForm(V vector, M matrix) {
        return quadraticForm(vector, matrix, vector);
    }

    default V normalize(V vector) {
        double mag = norm(vector);
        if (mag < 1e-8) throw new IllegalArgumentException("Can't normalize zero vector");
        return vector.scale(1 / mag);
    }
}

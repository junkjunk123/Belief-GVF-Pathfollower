package mathematics;

import manifold.ManifoldPoint;

public interface DifferentiableScalarFunction<P extends ManifoldPoint<P>> {
    double evaluate(P point);
}

package solver;

import manifold.BilinearForm;
import manifold.ManifoldPoint;
import manifold.RiemannianManifold;
import manifold.TangentVector;

public interface Path<Workspace extends RiemannianManifold<Pose, Vector, Matrix>,
        Pose extends ManifoldPoint<Pose>,
        Vector extends TangentVector<Pose, Vector>,
        Matrix extends BilinearForm<Vector>> {
    Vector getGradient(double t);
    Vector getTa
}

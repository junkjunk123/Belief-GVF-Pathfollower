package solver;

import manifold.ManifoldPoint;
import manifold.Matrix;
import manifold.RiemannianManifold;
import manifold.TangentVector;

public abstract class Workspace implements RiemannianManifold<Workspace.Pose, Workspace.Twist, Workspace.MatrixType> {
    public abstract static class Pose implements ManifoldPoint<Pose> {

    }

    public abstract static class Twist implements TangentVector<Pose, Twist> {

    }

    public abstract static class MatrixType implements Matrix<Twist> {

    }
}

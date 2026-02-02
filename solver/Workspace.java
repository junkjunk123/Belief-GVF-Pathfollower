package solver;

import manifold.ManifoldPoint;
import manifold.endomorphism.Endomorphism;
import manifold.RiemannianManifold;
import manifold.TangentVector;

public abstract class Workspace implements RiemannianManifold<Workspace.Pose, Workspace.Twist, Workspace.Matrix> {
    public abstract static class Pose implements ManifoldPoint<Pose> {

    }

    public abstract static class Twist implements TangentVector<Pose, Twist> {

    }

    public abstract static class Matrix implements Endomorphism<Twist> {

    }
}

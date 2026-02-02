package solver;

import belief.Belief;
import belief.BeliefVector;
import gvf.GVFPath;
import manifold.ManifoldPoint;
import manifold.ProductManifold;
import manifold.endomorphism.Endomorphism;
import manifold.RiemannianManifold;
import manifold.TangentVector;
import mathematics.VariationalFreeEnergyCalculator;

public class Solver<Workspace extends RiemannianManifold<Pose, Twist, Endomorphism<Twist>>,
        Pose extends ManifoldPoint<Pose>,
        Twist extends TangentVector<Pose, Twist>> {
    private final Workspace workspace;
    private final GVFPath<Workspace, Pose, Twist, Endomorphism<Twist>> path;
    private final Sensor<Pose, Twist> sensor;
    private Belief<Pose, Twist, Workspace> currentState;
    private double currentFreeEnergy;

    public Solver(Workspace workspace, GVFPath<Workspace, Pose, Twist,
            Endomorphism<Twist>> path, Sensor<Pose, Twist> sensor) {
        this.path = path;
        this.sensor = sensor;
        this.workspace = workspace;
    }

    public BeliefVector<Pose, Twist, Workspace> update() {

    }

    public void setCurrentState(Belief<Pose, Twist, Workspace> currentState) {
        this.currentState = currentState;
    }

    public void setStartPose(Pose pose, ProductManifold.ProductEndomorphism<Pose, Twist, Pose, Twist> initialCovariance) {
        this.currentState = new Belief<>(pose, path.evaluate(pose), initialCovariance, workspace);
    }

    public Belief<Pose, Twist, Workspace> getCurrentState() {
        return currentState;
    }

    public double variationalFreeEnergy(Belief<Pose, Twist, Workspace> belief) {
        return VariationalFreeEnergyCalculator.calcVariationalFreeEnergy(path, sensor, belief, workspace);
    }

    public double variationalFreeEnergy() {
        return currentFreeEnergy;
    }
}

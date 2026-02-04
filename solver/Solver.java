package solver;

import belief.Belief;
import gvf.GVFPath;
import manifold.ManifoldPoint;
import manifold.ProductManifold;
import manifold.endomorphism.Endomorphism;
import manifold.RiemannianManifold;
import manifold.TangentVector;
import mathematics.VariationalFreeEnergyCalculator;
import util.Timer;

public class Solver<Workspace extends RiemannianManifold<Pose, Twist, Endomorphism<Twist>>,
        Pose extends ManifoldPoint<Pose>,
        Twist extends TangentVector<Pose, Twist>> {
    private final Workspace workspace;
    private final GVFPath<Workspace, Pose, Twist> path;
    private final Sensor<Pose, Twist> sensor;
    private Belief<Pose, Twist, Workspace> currentState;
    private final Timer timer = new Timer();

    public Solver(Workspace workspace, GVFPath<Workspace, Pose, Twist> path, Sensor<Pose, Twist> sensor) {
        this.path = path;
        this.sensor = sensor;
        this.workspace = workspace;
    }

    public void update() {
        if (!timer.isStarted()) timer.start();
        double dt = timer.lapSeconds();
        Twist poseInnovation = workspace.log(currentState.muPose, sensor.read());
        Twist poseGradient = sensor.inverseCovariance().multiply(poseInnovation);
        Twist twistInnovation = path.evaluate(currentState.muPose).subtract(currentState.muTwist);
        Twist twistGradient = path.inverseCovariance().multiply(twistInnovation);
        Twist gvfJacobian = workspace.covariantDerivative(currentState.muPose, path).transpose().multiply(twistGradient);

        //Wasserstein Update
        Twist dMuPose = poseGradient.add(gvfJacobian).scale(-1);
        Twist dMuTwist = twistGradient.scale(-1);

        Endomorphism<Twist> dSigmaPose = sensor.inverseCovariance().multiply(currentState.sigma.one)
                .multiply(currentState.sigma.one)
                .scale(-2.0);
        Endomorphism<Twist> D = workspace.covariantDerivative(currentState.muPose, path);
        Endomorphism<Twist> dSigmaTwist = D.transpose().multiply(D)
                .multiply(currentState.sigma.two)
                .multiply(currentState.sigma.two)
                .scale(-2.0);

        Pose newPose = workspace.exp(currentState.muPose, dMuPose.scale(dt));
        Twist newTwist = currentState.muTwist.add(dMuTwist.scale(dt));
        ProductManifold.ProductEndomorphism<Pose, Twist, Pose, Twist> newSigma =
                currentState.sigma.add(new ProductManifold.ProductEndomorphism<>(dSigmaPose.scale(dt), dSigmaTwist.scale(dt)));
        setCurrentState(new Belief<>(newPose, newTwist, newSigma, workspace));
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
}

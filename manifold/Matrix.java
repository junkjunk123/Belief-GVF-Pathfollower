package manifold;

public interface Matrix<VectorType extends TangentVector<?, VectorType>> {
    double determinant();
    Matrix<VectorType> inverse();
    VectorType multiply(VectorType vector);
}

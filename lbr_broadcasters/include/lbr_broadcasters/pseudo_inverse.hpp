#ifndef LBR_BROADCASTERS__PSEUDO_INVERSE_HPP_
#define LBR_BROADCASTERS__PSEUDO_INVERSE_HPP_

#include <Eigen/Core>

// damped least squares solutions:
// http://graphics.cs.cmu.edu/nsp/course/15-464/Spring11/handouts/iksurvey.pdf
template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
damped_leaste_squares(const MatT &mat, typename MatT::Scalar lambda =
                                           typename MatT::Scalar{2e-1}) // choose appropriately
{
  typedef typename MatT::Scalar Scalar;
  auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto &singularValues = svd.singularValues();
  Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> dampedSingularValuesInv(
      mat.cols(), mat.rows());
  dampedSingularValuesInv.setZero();
  for (unsigned int i = 0; i < singularValues.size(); ++i) {
    dampedSingularValuesInv(i, i) =
        singularValues(i) / (singularValues(i) * singularValues(i) + lambda * lambda);
  }
  return svd.matrixV() * dampedSingularValuesInv * svd.matrixU().adjoint();
};

#endif // LBR_BROADCASTERS__PSEUDO_INVERSE_HPP_

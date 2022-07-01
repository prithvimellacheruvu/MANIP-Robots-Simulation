#include <visp3/core/vpThetaUVector.h>

int main()
{
  vpHomogeneousMatrix M(0, 0, 1, vpMath::rad(10), vpMath::rad(20), vpMath::rad(30));

  vpPoseVector pose;

  auto u = M.getRotationMatrix().getThetaUVector().getU();
  auto theta = M.getRotationMatrix().getThetaUVector().getTheta();
  std::cout << "pose: " << pose.buildFrom(M) << std::endl;
  std::cout << "theta: " << theta << std::endl;
  std::cout << "u    : " << u.t() << std::endl;
  std::cout << "t   : " << M.getTranslationVector().t() << std::endl;
  std::cout << "u * theta: " << u * theta << std::endl;
  std::cout << "theta u vector: " << M.getRotationMatrix().getThetaUVector() << std::endl;
  std::cout << "Homo Mat: " << M << std::endl;
  std::cout << "Rot Mat: " << M.getRotationMatrix() << std::endl;
}

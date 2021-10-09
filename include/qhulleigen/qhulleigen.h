#ifndef QHULLEIGEN_H
#define QHULLEIGEN_H

extern "C" {
#include <qhull/qhull_a.h>
}

#include <Eigen/Eigen>

namespace qhulleigen{
  // In = [v1 v2 v3 v4 ..]
  bool convexhull(const Eigen::MatrixXd& In, Eigen::MatrixXd& Out, std::vector<std::vector<int> >& Face, bool calc_face=true);

  bool convexhull(const Eigen::MatrixXd& In, Eigen::MatrixXd& Out);
};


#endif

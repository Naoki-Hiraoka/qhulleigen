#ifndef QHULLEIGEN_H
#define QHULLEIGEN_H

#include <Eigen/Eigen>


namespace qhulleigen{
  // In = [v1 v2 v3 v4 ..]
  bool convexhull(const Eigen::MatrixXd& In, Eigen::MatrixXd& Out, std::vector<std::vector<int> >& Face, bool calc_face=true);

  bool convexhull(const Eigen::MatrixXd& In, Eigen::MatrixXd& Out, Eigen::MatrixXi& Face, bool calc_face=true);

  bool convexhull(const Eigen::MatrixXd& In, Eigen::MatrixXd& Out);
};


#endif

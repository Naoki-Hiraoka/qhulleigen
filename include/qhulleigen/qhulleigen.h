#ifndef QHULLEIGEN_H
#define QHULLEIGEN_H

#include <Eigen/Eigen>


namespace qhulleigen{
  // In = [v1 v2 v3 v4 ..]
  // Out = [v1 v2 v3 v4 ..]
  // Inの頂点数が4未満であるなどconvex hullの計算に失敗した場合はfalseを返し、OutにInがそのまま入る. そのときのFaceの値は未定義
  bool convexhull(const Eigen::MatrixXd& In, Eigen::MatrixXd& Out, std::vector<std::vector<int> >& Face, bool calc_face=true);

  bool convexhull(const Eigen::MatrixXd& In, Eigen::MatrixXd& Out, Eigen::MatrixXi& Face, bool calc_face=true);

  bool convexhull(const Eigen::MatrixXd& In, Eigen::MatrixXd& Out);
};


#endif

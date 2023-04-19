#include <qhulleigen/qhulleigen.h>
#include <iostream>

/*
  qhullはマルチスレッド対応していないので、qhull_rを使っている
  参考:
    https://github.com/PointCloudLibrary/pcl/pull/4540
    http://www.qhull.org/html/qh-code.htm
 */

extern "C" {
#include <libqhull_r/qhull_ra.h>
}

namespace qhulleigen{
  bool convexhull(const Eigen::MatrixXd& In, Eigen::MatrixXd& Out, std::vector<std::vector<int> >& Face, bool calc_face){
    // convex hull by qhull.
    int numVertices = In.cols();
    int dim = In.rows();
    double points[numVertices*dim];
    for (int i=0; i<numVertices; i++){
      for(size_t j=0;j<dim;j++){
        points[i*dim+j] = In(j,i);
      }
    }

    qhT qh_qh;
    qhT* qh = &qh_qh;
    QHULL_LIB_CHECK
    qh_zero(qh, stderr);

    char flags[250];
    boolT ismalloc = False;
    sprintf(flags,"qhull Qt Tc Fx");
    if (qh_new_qhull (qh, dim,numVertices,points,ismalloc,flags,NULL,stderr)) {
      qh_freeqhull(qh, !qh_ALL);
      int curlong, totlong;
      qh_memfreeshort (qh, &curlong, &totlong);
      return false;
    }

    qh_triangulate(qh);
    qh_vertexneighbors(qh);

    std::vector<Eigen::VectorXd> hull;
    int index[numVertices];
    int vertexIndex = 0;
    vertexT *vertex;
    FORALLvertices {
      int p = qh_pointid(qh, vertex->point);
      index[p] = vertexIndex;
      hull.push_back(In.col(p));
      vertexIndex++;
    }
    Out.resize(dim,hull.size());
    for(size_t i=0;i<hull.size();i++){
      Out.col(i) = hull[i];
    }

    if(calc_face){
      Face.clear();

      facetT *facet;
      int num = qh->num_facets;
      int triangleIndex = 0;
      FORALLfacets {
        int j = 0;
        std::vector<int> p;
        p.reserve(dim);
        setT *vertices;
        if (dim==3) vertices = qh_facet3vertex (qh, facet); //時計回りになる
        else vertices = facet->vertices; // dim=3以外は時計回りが定義できないので頂点のvertexを順番は適当でそのまま返す
        vertexT **vertexp;
        FOREACHvertexreverse12_ (vertices) {
          if (j<dim) {
            p.push_back(index[qh_pointid(qh, vertex->point)]);
          } else {
            fprintf(stderr, "extra vertex %d\n",j);
          }
          j++;
        }
        Face.push_back(p);
      }
    }

    qh_freeqhull(qh, !qh_ALL);
    int curlong, totlong;
    qh_memfreeshort (qh, &curlong, &totlong);
    return true;
  }

  bool convexhull(const Eigen::MatrixXd& In, Eigen::MatrixXd& Out){
    std::vector<std::vector<int> > Face;
    return convexhull(In, Out, Face, false);
  }
}

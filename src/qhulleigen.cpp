#include <qhulleigen/qhulleigen.h>

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
    char flags[250];
    boolT ismalloc = False;
    sprintf(flags,"qhull Qt Tc Fx");
    if (qh_new_qhull (dim,numVertices,points,ismalloc,flags,NULL,stderr)) return false;

    qh_triangulate();
    qh_vertexneighbors();

    std::vector<Eigen::VectorXd> hull;
    int index[numVertices];
    int vertexIndex = 0;
    vertexT *vertex;
    FORALLvertices {
      int p = qh_pointid(vertex->point);
      index[p] = vertexIndex;//なぜか分からないがpの値が数百あるので，多分ここでメモリ確保違反をしている
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
      int num = qh num_facets;
      int triangleIndex = 0;
      FORALLfacets {
        int j = 0;
        std::vector<int> p;
        p.reserve(dim);
        setT *vertices;
        if (dim==3) vertices = qh_facet3vertex (facet); //時計回りになる
        else vertices = facet->vertices;
        vertexT **vertexp;
        FOREACHvertexreverse12_ (vertices) {
          if (j<dim) {
            p.push_back(index[qh_pointid(vertex->point)]);
          } else {
            fprintf(stderr, "extra vertex %d\n",j);
          }
          j++;
        }
        Face.push_back(p);
      }
    }

    qh_freeqhull(!qh_ALL);
    int curlong, totlong;
    qh_memfreeshort (&curlong, &totlong);

    return true;
  }

  bool convexhull(const Eigen::MatrixXd& In, Eigen::MatrixXd& Out){
    std::vector<std::vector<int> > Face;
    return convexhull(In, Out, Face, false);
  }
}

#ifndef GLUWRAPPER_H
#define GLUWRAPPER_H

#include <nicp_viewer/nicp_qglviewer.h>
#include <nicp_viewer/drawable_points.h>

class GLUWrapper {
public:
    static GLUquadricObj* getQuadradic() {
        static GLUWrapper inst;
        return inst._quadratic;
    }
protected:
    GLUWrapper() {
        _quadratic = gluNewQuadric();
        gluQuadricNormals(_quadratic, GLU_SMOOTH);
    }
    ~GLUWrapper() {
        gluDeleteQuadric(_quadratic);
    }

    GLUquadricObj *_quadratic;;
};

#endif // GLUWRAPPER_H

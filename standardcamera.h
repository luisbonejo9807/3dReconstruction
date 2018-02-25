#ifndef STANDARDCAMERA_H
#define STANDARDCAMERA_H

#include <nicp_viewer/nicp_qglviewer.h>
#include <nicp_viewer/drawable_points.h>

using namespace nicp_viewer;

class StandardCamera: public qglviewer::Camera {
public:
    StandardCamera(): _standard(true) {}

    qreal zNear() const {
        if(_standard) { return qreal(0.001f); }
        else { return Camera::zNear(); }
    }

    qreal zFar() const {
        if(_standard) { return qreal(10000.0f); }
        else { return Camera::zFar(); }
    }

    bool standard() const { return _standard; }
    void setStandard(bool s) { _standard = s; }

protected:
    bool _standard;
};

#endif // STANDARDCAMERA_H

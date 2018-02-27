#ifndef NICPTRACKERAPPVIEWER_H
#define NICPTRACKERAPPVIEWER_H

#include <nicp_viewer/nicp_qglviewer.h>
#include <nicp_viewer/drawable_points.h>

#include "nicp/cloud.h"

#include "standardcamera.h"
#include "gluwrapper.h"

using namespace nicp_viewer;

class NICPTrackerAppViewer: public NICPQGLViewer {
public:
    NICPTrackerAppViewer(QWidget *parent = 0,
                         const QGLWidget *shareWidget = 0,
                         Qt::WindowFlags flags = 0): NICPQGLViewer(parent, shareWidget, flags) {
        _spin = _spinOnce = false;
        _needRedraw = true;
        _currentTransform = Eigen::Isometry3f::Identity();
        _drawableCurrentCloud = 0;
        _drawableReferenceScene = 0;
    }
    ~NICPTrackerAppViewer() {}

    virtual void keyPressEvent(QKeyEvent* e) {
        NICPQGLViewer::keyPressEvent(e);
        // Start tracking
        if((e->key() == Qt::Key_T)) {
            std::cout << "[INFO]: starting tracking" << std::endl;
            _spin = true;
        }
        // Align next depth image
        else if((e->key() == Qt::Key_N)) {
            std::cout << "[INFO]: aligning next depth image" << std::endl;
            _spinOnce = true;
        }
        // Stop tracking
        else if((e->key() == Qt::Key_P)) {
            std::cout << "[INFO]: stop tracking" << std::endl;
            _spin = _spinOnce = false;
        }
        else {}
    }

    virtual void init() {
        NICPQGLViewer::init();
        setBackgroundColor(QColor::fromRgb(100, 100, 100));
        setMouseBinding(Qt::ControlModifier, Qt::LeftButton, RAP_FROM_PIXEL);
        qglviewer::Camera* oldcam = camera();
        qglviewer::Camera* cam = new StandardCamera();
        setCamera(cam);
        cam->setPosition(qglviewer::Vec(0.0f, 0.0f, 0.0f));
        cam->setUpVector(qglviewer::Vec(0.0f, -1.0f, 0.0f));
        cam->lookAt(qglviewer::Vec(0.0f, 0.0f, 1.0f));
        delete oldcam;
    }

    void drawSphere(GLfloat radius) {
        glNormal3f(0.0f, 0.0f, -1.0f);
        gluSphere(GLUWrapper::getQuadradic(), radius, 32, 32);
    }

    void drawPyramid(GLfloat pyrH, GLfloat pyrW) {
        glBegin(GL_TRIANGLES);
        glVertex3f(pyrW, pyrW, pyrH);
        glVertex3f(pyrW, -pyrW, pyrH);
        glVertex3f(-pyrW, -pyrW, pyrH);
        glVertex3f(-pyrW, -pyrW, pyrH);
        glVertex3f(-pyrW, pyrW, pyrH);
        glVertex3f(pyrW, pyrW, pyrH);
        glEnd();

        glBegin(GL_TRIANGLES);
        glVertex3f(pyrW, -pyrW, pyrH);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(pyrW, pyrW, pyrH);
        glVertex3f(pyrW, pyrW, pyrH);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(-pyrW, pyrW, pyrH);
        glVertex3f(-pyrW, pyrW, pyrH);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(-pyrW, -pyrW, pyrH);
        glVertex3f(-pyrW, -pyrW, pyrH);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(pyrW, -pyrW, pyrH);
        glEnd();
    }

    void drawPyramidWireframe(float pyrH, float pyrW) {
        glLineWidth(3.0f);
        glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
        glBegin(GL_LINE_LOOP);
        glVertex3f(pyrW,-pyrW,pyrH);
        glVertex3f(pyrW,pyrW,pyrH);
        glVertex3f(-pyrW,pyrW,pyrH);
        glVertex3f(-pyrW,-pyrW,pyrH);
        glEnd();

        glBegin(GL_LINES);
        glVertex3f(pyrW,-pyrW,pyrH);
        glVertex3f(0.0,0.0,0.0);
        glVertex3f(pyrW,pyrW,pyrH);
        glVertex3f(0.0,0.0,0.0);
        glVertex3f(-pyrW,pyrW,pyrH);
        glVertex3f(0.0,0.0,0.0);
        glVertex3f(-pyrW,-pyrW,pyrH);
        glVertex3f(0.0,0.0,0.0);
        glEnd();
    }

    virtual void draw() {
        NICPQGLViewer::draw();

        glColor4f(0.0, 0.0f, 1.0f, 1.0f);
        drawAxis(0.2f);

        float radius = 0.02f;
        // Draw current estimated camera frame
        if(_estimatedPoses.size() > 0) {
            glPushMatrix();
            glColor4f(1.0, 0.0f, 0.0f, 1.0f);
            glMultMatrixf(_estimatedPoses[_estimatedPoses.size() - 1].data());
            float pyrH = 0.1f;
            float pyrW = 0.05f;
            drawPyramidWireframe(pyrH, pyrW);
            glPopMatrix();
        }

        // Draw current groundtruth camera frame
        if(_groundtruthPoses.size() > 0) {
            glPushMatrix();
            glColor4f(0.0, 1.0f, 0.0f, 1.0f);
            glMultMatrixf(_groundtruthPoses[_groundtruthPoses.size() - 1].data());
            float pyrH = 0.1f;
            float pyrW = 0.05f;
            drawPyramidWireframe(pyrH, pyrW);
            glPopMatrix();
        }

        // Draw estimated trajectory
        for(size_t i = 0; i < _estimatedPoses.size(); ++i) {
            glPushMatrix();
            glColor4f(1.0, 0.0f, 0.0f, 1.0f);
            glMultMatrixf(_estimatedPoses[i].data());
            drawSphere(radius);
            glPopMatrix();
        }

        // Draw groundtruth trajectory
        for(size_t i = 0; i < _groundtruthPoses.size(); ++i) {
            glPushMatrix();
            glColor4f(0.0, 1.0f, 0.0f, 1.0f);
            glMultMatrixf(_groundtruthPoses[i].data());
            drawSphere(radius);
            glPopMatrix();
        }

        _needRedraw = false;
    }

    void updateReferenceScene(Cloud* referenceScene_, Eigen::Isometry3f transform_) {
        if(!_drawableReferenceScene) {
            _drawableReferenceScene = new DrawablePoints(transform_,
                                                         new GLParameterPoints(2.0f, Eigen::Vector4f(1.0f, 0.5f, 0.0f, 0.75f)),
                                                         &referenceScene_->points(), &referenceScene_->normals(), &referenceScene_->rgbs());
            addDrawable(_drawableReferenceScene);
        }
        else {
            _drawableReferenceScene->setTransformation(transform_);
            DrawablePoints* dp = dynamic_cast<DrawablePoints*>(_drawableReferenceScene);
            if(dp) { dp->updatePointDrawList(); }
        }
        _needRedraw = true;
    }

    void updateCurrentCloud(Cloud* currentCloud_, Eigen::Isometry3f transform_) {
        if(!_drawableCurrentCloud) {
            _drawableCurrentCloud = new DrawablePoints(transform_,
                                                       new GLParameterPoints(2.0f, Eigen::Vector4f(0.0f, 0.0f, 1.0f, 1.0f)),
                                                       &currentCloud_->points(), &currentCloud_->normals(), &currentCloud_->rgbs());
            addDrawable(_drawableCurrentCloud);
        }
        else {
            _drawableCurrentCloud->setTransformation(transform_);
            DrawablePoints* dp = dynamic_cast<DrawablePoints*>(_drawableCurrentCloud);
            if(dp) {
                dp->setPointsAndNormals(&currentCloud_->points(), &currentCloud_->normals());
                dp->updatePointDrawList();
            }
        }
        _currentCloud = currentCloud_;
        _estimatedPoses.push_back(transform_);
        _currentTransform = transform_;
        _needRedraw = true;
    }

    void resetReferenceScene() {
        DrawablePoints* dp = dynamic_cast<DrawablePoints*>(_drawableReferenceScene);
        GLParameterPoints* pp = dynamic_cast<GLParameterPoints*>(_drawableReferenceScene->parameter());
        if(dp && pp) {
            pp->setColor(Eigen::Vector4f(1.0f, 0.5f, 0.0f, 1.0f));
            dp->updatePointDrawList();
        }
        _drawableReferenceScene = 0;
        _needRedraw = true;
    }

    void addGroundtruthPose(const Eigen::Isometry3f pose) {
        _groundtruthPoses.push_back(pose);
        _needRedraw = true;
    }


    inline bool needRedraw() const { return _needRedraw; }
    inline bool spin() const { return _spin; }
    inline bool spinOnce() {
        bool ret = _spinOnce;
        _spinOnce = false;
        return ret;
    }

protected:
    bool _spin;
    bool _spinOnce;
    bool _needRedraw;
    Eigen::Isometry3f _currentTransform;
    std::vector<Eigen::Isometry3f> _groundtruthPoses;
    std::vector<Eigen::Isometry3f> _estimatedPoses;
    Cloud* _currentCloud;
    Drawable* _drawableReferenceScene;
    Drawable* _drawableCurrentCloud;
};

#endif // NICPTRACKERAPPVIEWER_H

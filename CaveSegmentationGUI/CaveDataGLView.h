#pragma once
#include "GLView.h"
#include <qglfunctions.h>
#include "CaveGLData.hpp"
#include <QTimer>

class CaveDataGLView : public GLView, QGLFunctions {
	

public:
	CaveDataGLView(CaveGLData& data, QWidget * parent = Q_NULLPTR);
	~CaveDataGLView();

protected slots:
	void meshChanged();
#ifdef NSIGHT_COMPATIBLE
	void render();
#endif

private:
	virtual void paintGL();
	virtual void initializeGL();

	CaveGLData& data;
#ifdef NSIGHT_COMPATIBLE
	QTimer timer;
#endif
};

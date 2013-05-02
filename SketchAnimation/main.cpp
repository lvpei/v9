#include <QApplication>
#include "mainwindow.h"

 int main(int argc, char *argv[])
 {
     QApplication app(argc, argv);

	 QGLFormat f = QGLFormat::defaultFormat();
	 f.setSampleBuffers(true);
	 f.setSamples(8);
	 f.setStencil(true);
	 QGLFormat::setDefaultFormat(f);

     MainWindow mainwin;
     mainwin.show();
     return app.exec();
 }

#ifndef CQCamera3DShaderProgram_H
#define CQCamera3DShaderProgram_H

#include <QOpenGLShaderProgram>
#include <QString>

class CQGLBuffer;

namespace CQTclModel3DView {

class App;

class ShaderProgram : public QOpenGLShaderProgram {
 public:
  ShaderProgram(App *app);

  void addShaders(const QString &vertex, const QString &fragment);

  void addVertexShader  (const QString &name);
  void addFragmentShader(const QString &name);
  void addGeometryShader(const QString &name);

  CQGLBuffer *buffer() const { return buffer_; }

  CQGLBuffer *createBuffer();

 private:
  App*        app_    { nullptr };
  CQGLBuffer* buffer_ { nullptr };
};

}

#endif

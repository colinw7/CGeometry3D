#ifndef CQCamera3DShaderProgram_H
#define CQCamera3DShaderProgram_H

#include <QOpenGLShaderProgram>
#include <QString>

class CQGLBuffer;

namespace CQTclParticle3D {

class App;

class ShaderProgram : public QOpenGLShaderProgram {
 public:
  ShaderProgram(App *app);

  void addShaders(const QString &vertex, const QString &fragment);

  void addVertexShader  (const QString &name);
  void addFragmentShader(const QString &name);

  CQGLBuffer *createBuffer();

 private:
  App* app_ { nullptr };
};

}

#endif

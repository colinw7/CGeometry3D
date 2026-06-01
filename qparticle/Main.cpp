#include <Main.h>
#include <App.h>

#include <CQImage.h>
#include <CImportBase.h>
#include <CGeom3DType.h>

#include <QApplication>

int
main(int argc, char **argv)
{
  QApplication qapp(argc, argv);

  CQImage::setPrototype();

  //---

  std::vector<std::string> filenames;

  for (int i = 1; i < argc; ++i) {
    if (argv[i][0] == '-') {
      auto arg = QString(&argv[i][1]);

      if (arg == "h" || arg == "help") {
        std::cerr << "CQTclParticle3D <file>\n";
        return 0;
      }
      else {
        std::cerr << "Invalid option " << argv[i] << "\n";
        exit(1);
      }
    }
    else {
      filenames.push_back(argv[i]);
    }
  }

  //---

  auto *app = new CQTclParticle3D::App;

  app->show();

  //---

  for (auto &filename : filenames)
    app->execFile(filename);

  //---

  qapp.exec();
}

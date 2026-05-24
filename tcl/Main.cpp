#include <Main.h>
#include <App.h>

int
main(int argc, char **argv)
{
  auto *app = new CTclGeometry3D::App;

  std::vector<std::string> filenames;

  for (int i = 1; i < argc; ++i) {
    if (argv[i][0] == '-') {
      auto arg = std::string(&argv[i][1]);

      if (arg == "h" || arg == "help") {
        std::cout << "CTclGeometry3D <file>\n";
        exit(0);
      }
      else {
        std::cerr << "Invalid arg '" << argv[i] << "'\n";
        exit(1);
      }
    }
    else
      filenames.push_back(argv[i]);
  }

  for (auto &filename : filenames)
    app->execFile(filename);

  return 0;
}

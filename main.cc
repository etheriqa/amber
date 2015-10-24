#include "cli/application.h"

int main(int argc, char **argv) {
  return amber::cli::Application(argc, argv).run();
}

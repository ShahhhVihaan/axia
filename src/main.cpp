#include "renderer.h"

int main(int argc, const char** argv) {
  // if (argc != 2) {
  //   std::printf("USAGE: ./main <model.xml>\n");
  //   return 1;
  // }
  
  // hardcode quadruped for now 
  // TODO: create some kind of config/registry
  std::string quadruped_model = "menagerie-link/unitree_a1/scene.xml";

  Renderer renderer;
  if (!renderer.load(quadruped_model)) {
    return 1;
  }

  renderer.run();
  return 0;
}

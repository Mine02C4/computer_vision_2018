#include "image_manager.hpp"

fs::path ImageManager::root = "images";

std::tuple<std::string, std::string> ImageManager::GetStereoVideoPath(
    std::string basename) {
  auto left_path = root / (basename + "_left.mp4");
  auto right_path = root / (basename + "_right.mp4");
  return std::make_tuple(left_path.string(), right_path.string());
}

std::string ImageManager::GetImagePath(std::string filename) {
  return (root / filename).string();
}
